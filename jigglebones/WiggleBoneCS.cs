using Godot;
using System;

[Tool]
public class WiggleBoneCS : BoneAttachment
{
    [Export] public bool enable = true;
    [Export(PropertyHint.Range, "0.01,10,0.01")] public float stiffness = 0.1f;
    [Export(PropertyHint.Range, "0,1,0.01")] public float damping = 0.1f;
    [Export(PropertyHint.Range, "0,10,0.01")] public float gravity_scale = 1.0f;
    [Export] public bool use_gravity = false;
    [Export] public Vector3 gravity = new Vector3(0.0f, -9.81f, 0.0f);
    [Export(PropertyHint.Range, "0.0,100.0,1.0")] public float apply_percent = 80.0f;
    [Export(PropertyHint.Range, "0.0,1.0,0.001")] public float length = 0.1f;
    [Export(PropertyHint.Range, "0.01,1,0.01")] public float max_distance = 0.1f;

    [Export(PropertyHint.Range, "0, 10")] public int skip_frame = 0;

    public string bone_name = "";
    private int frame = 0;

    private PointMass _point_mass = new PointMass();
    private Vector3 _acceleration = Vector3.Zero;
    private Vector3 _prev_mass_center = Vector3.Zero;
    private Vector3 _prev_velocity = Vector3.Zero;
    private bool _should_reset = true;
    private Basis _global_to_pose = new Basis();

    private Vector3 prev_pos = Vector3.Zero;

    public override void _Ready()
    {
        if (enable)
        {
            SetAsToplevel(true); // Ignore parent transformation
            prev_pos = GlobalTransform.origin;
        }
    }

    public override void _PhysicsProcess(float delta)
    {
        if (!enable)
            return;
        frame += 1;
        if (frame > skip_frame)
        {
            frame = 0;
            // Process(delta * (float)Convert.ToDecimal(1 + skip_frame));
            Process(delta * (float)(1 + skip_frame));

        }
    }

    private void Process(float delta)
    {
        // if (!enable)
        //     return;

        Skeleton skeleton;
        int bone_id;
        int bone_id_parent;
        bone_name = BoneName;

        if (Engine.EditorHint)
        {
            skeleton = (Skeleton)GetParent();
            if (! (skeleton is Skeleton))
            {
                jiggleprint("Jigglebone must be a direct child of a Skeleton node");
                return;
            }
            if (bone_name == "")
            {
                jiggleprint("Please enter a bone name");
                return;
            }
            bone_id = skeleton.FindBone(bone_name);
            if (bone_id == -1)
            {
                jiggleprint("Unknown bone " + bone_name + " - please enter a valid bone name");
                return;
            }
            bone_id_parent = skeleton.GetBoneParent(bone_id);
        }
        else
        {
            skeleton = (Skeleton)GetParent();
            bone_id = skeleton.FindBone(bone_name);
            bone_id_parent = skeleton.GetBoneParent(bone_id);
        }

        // Wiggle bone
        Transform bonePose = skeleton.GetBonePose(bone_id);
        if (bone_id_parent >= 0)
        {
            bonePose = skeleton.GetBoneGlobalPose(bone_id_parent) * skeleton.GetBoneRest(bone_id) * bonePose;
        }

        Transform globalBonePose = skeleton.GlobalTransform * bonePose;
        _global_to_pose = globalBonePose.basis.Inverse();

        Vector3 newAcceleration = _update_acceleration(globalBonePose, delta);
        _acceleration = _acceleration.LinearInterpolate(newAcceleration, 0.5f);

        //adjust for varying framerates
        //this is only an approximation
        // float deltaFactor = Mathf.Log(delta * 60.0f) / Mathf.Log(2.0f) + 1.0f;
        // _acceleration /= Mathf.Clamp(deltaFactor, 1.0f, 3.0f);

        _acceleration /= (float)(skip_frame) + 1.0f;

        Transform pose = bonePose * _pose();
        
        skeleton.SetBoneGlobalPoseOverride(bone_id, pose, apply_percent / 100.0f, true);

        _solve(_global_to_pose.Orthonormalized(), _acceleration, delta);
    }

    private void jiggleprint(string text)
    {
        GD.Print($"[Jigglebones] Error: {text}");
    }


    private Transform _pose()
    {
        Transform pose = new Transform();
        pose = Transform.Identity;
        
        float k = max_distance * 0.5f;
        Vector3 massConstrained = _clamp_length_soft(_point_mass.P, 0.0f, max_distance, k);
        
        pose.origin = massConstrained;

        return pose;
    }

    private Vector3 _clamp_length_soft(Vector3 v, float minLength, float maxLength, float k)
    {
        return v.Normalized() * _smin(Mathf.Max(minLength, v.Length()), maxLength, k);
    }

    // https://iquilezles.org/articles/smin/
    private float _smin(float a, float b, float k)
    {
        float h = Mathf.Max(0.0f, k - Mathf.Abs(a - b));
        return Mathf.Min(a, b) - h * h / (4.0f * k);
    }

    private Vector3 _update_acceleration(Transform globalBonePose, float delta)
    {
        Vector3 massCenter = Vector3.Zero;

        massCenter = globalBonePose.Xform(massCenter);
        Vector3 deltaMassCenter = _prev_mass_center - massCenter;
        _prev_mass_center = massCenter;

        if (_should_reset)
        {
            deltaMassCenter = Vector3.Zero;
        }

        Vector3 globalVelocity = deltaMassCenter / delta;
        _acceleration = globalVelocity - _prev_velocity;
        _prev_velocity = globalVelocity;

        if (_should_reset)
        {
            _acceleration = Vector3.Zero;
            _should_reset = false;
        }

        return _acceleration;
    }

    private void _solve(Basis globalToLocal, Vector3 acceleration, float delta)
    {
        Vector3 globalForce = gravity * gravity_scale;
        if (!use_gravity)
        {
            globalForce = Vector3.Zero;
        }

        Vector3 localForce = globalToLocal * globalForce;

        float massDistance = length;
        Vector3 localAcc = globalToLocal * acceleration;
        _point_mass.Accelerate(localAcc, delta);
        _point_mass.ApplyForce(localForce);
        _point_mass.Solve(stiffness, damping, delta, (float)(skip_frame + 1));
    }

    private class PointMass
    {
        public Vector3 P = Vector3.Zero;
        public Vector3 V = Vector3.Zero;
        public Vector3 A = Vector3.Zero;

        public void Solve(float stiffness, float damping, float delta, float factor = 1.0f)
        {
            //ineria
            // V = V * (1.0f - damping) + A * delta;
            // P += V;
            P += V * Mathf.Clamp(1.0f - damping, 0.0f, 1.0f) + (A * delta);
            A = Vector3.Zero;
            V = V * Mathf.Clamp(1.0f - damping * factor, 0.0f, 1.0f) + (A * delta);
            // constraint
            V -= P * stiffness * factor;
        }

        public void Accelerate(Vector3 acc, float delta)
        {
            V += acc * delta;
        }

        public void ApplyForce(Vector3 force)
        {
            A += force;
        }

        public void Reset()
        {
            P = Vector3.Zero;
            V = Vector3.Zero;
            A = Vector3.Zero;
        }
    }
}
