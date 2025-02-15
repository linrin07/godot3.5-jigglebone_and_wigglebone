using Godot;
using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;

[Tool]
public partial class JiggleBoneCS : Spatial
{
    // Declare member variables here. Examples:
    public enum Axis
    {
        X_Plus, Y_Plus, Z_Plus, X_Minus, Y_Minus, Z_Minus
    }

    [Export]
    public bool enable {get; set;} = true;
    [Export]
    public string bone_name {get; set;} = "";
    [Export(PropertyHint.Range, "0.01, 10, 0.01")]
    public float stiffness  {get; set;} = 0.1f;
    [Export(PropertyHint.Range, "0.01, 10, 0.01")]
    public float damping  {get; set;} = 0.0f;
    [Export(PropertyHint.Range, "0.01, 10, 0.01")]
    public float gravity_scale  {get; set;} = 1.0f;
    [Export]
    public bool use_gravity  {get; set;} = false;
    [Export]
    public Vector3 gravity  {get; set;} = new Vector3(0.0f, -9.81f, 0.0f);
    [Export(PropertyHint.Enum, "Axis")]
    public Axis forward_axis {get; set;} = Axis.Z_Minus;
    [Export(PropertyHint.Range, "0.0, 1.0, 0.0001")]
    public float bone_y_plus  {get; set;} = 0.1f;
    [Export(PropertyHint.Range, "0.0, 100.0, 1.0")]
    public float apply_percent {get; set;} = 80.0f;
    [Export(PropertyHint.Range, "0.0, 100.0, 0.01")]
    public float limitation {get; set;} = 10.0f;

    [Export]
    public NodePath collision_shape;
    public CollisionShape collision_sphere;
    public List<CollisionShape> collision_spheres = new List<CollisionShape>();

    private Vector3 prev_pos = new Vector3();

    // private float rest_length = 1.0f;

    [Export]
    private int skip_frame = 0;
    private int frame = 0;

    private void jiggleprint(String text){
        GD.Print("[Jigglebones] Error: " + text);
    }

    private Vector3 get_bone_forward_local() {
        switch (forward_axis)
        {
            case Axis.X_Plus: return new Vector3(1.0f,0.0f,0.0f);
            case Axis.Y_Plus: return new Vector3(0.0f,1.0f,0.0f);
            case Axis.Z_Plus: return new Vector3(0.0f,0.0f,1.0f);
            case Axis.X_Minus: return new Vector3(-1.0f,0.0f,0.0f);
            case Axis.Y_Minus: return new Vector3(0.0f,-1.0f,0.0f);
            case Axis.Z_Minus: return new Vector3(0.0f,0.0f,-1.0f);
            default: return new Vector3(0.0f, 1.0f, 0.0f);
        }
    }

    private void SetCollisionShape(NodePath path){
        if (path is null)
        {
            return;
        }

        collision_shape = path;
        collision_sphere = (CollisionShape)GetNodeOrNull(path);
        if (collision_sphere is null){
            return;
        }
        if (collision_sphere.Shape is SphereShape)
        {
            add_collision_shape(collision_sphere);
            return;
        }
        else
        {
            collision_sphere = null;
        }
    }

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        if (enable)
        {
            SetAsToplevel(true);
            prev_pos = GlobalTransform.origin;
            SetCollisionShape(collision_shape);
        }
    }

    [Obsolete]
    public void JiggleBoneProcess(float delta)
    {
        if (! enable)
        {
            return;
        }

        Skeleton skeleton;
        int bone_id;
        int bone_id_parent;

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


        // # Note:
        // # Local space = local to the bone
        // # Object space = local to the skeleton (confusingly called "global" in get_bone_global_pose)
        // # World space = global
        
        // # See https://godotengine.org/qa/7631/armature-differences-between-bones-custom_pose-transform
        Transform bone_transf_obj = skeleton.GetBoneGlobalPose(bone_id); // Object space bone pose
        Transform bone_transf_world = skeleton.GlobalTransform * bone_transf_obj;
        
        Transform bone_transf_rest_local = skeleton.GetBoneRest(bone_id);
        Transform bone_transf_rest_obj = skeleton.GetBoneGlobalPose(bone_id_parent) * bone_transf_rest_local; 
        Transform bone_transf_rest_world = skeleton.GlobalTransform * bone_transf_rest_obj;

        // ############### Integrate velocity (Verlet integration) ##############	
        
        // # If not using gravity, apply force in the direction of the bone (so it always wants to point "forward")
        Vector3 grav = bone_transf_rest_world.basis.Xform(Vector3.Down).Normalized() * 9.81f;
        Vector3 vel = (GlobalTransform.origin - prev_pos) / delta;
        if (float.IsNaN(vel.x))
        {
            vel = Vector3.Zero;
            return;
        }

        if (use_gravity)
        {
            grav = gravity * gravity_scale;
        }

        grav *= stiffness;
        vel += grav;
        vel -= vel * damping * delta; //Damping
        vel = vel.LimitLength(limitation);

        prev_pos = GlobalTransform.origin;
        GlobalPosition = GlobalTransform.origin + vel * delta;

        // ############### Solve distance constraint ##############

        Vector3 goal_pos = skeleton.ToGlobal(skeleton.GetBoneGlobalPose(bone_id).origin);
        Vector3 new_pos_clamped = goal_pos + (GlobalTransform.origin - goal_pos).Normalized() * bone_y_plus;
        GlobalPosition = new_pos_clamped;
        /*
        if (!(collision_sphere is null))
        {
            Vector3 test_vec = GlobalTransform.origin - collision_sphere.GlobalTransform.origin;
            float distance = test_vec.Length() - (collision_sphere.Shape as SphereShape).Radius;
            if (distance < 0)
            {
                GlobalPosition -= test_vec.Normalized() * distance;
            }
        }*/
        if (collision_spheres.Count > 0)
        {   
            // Vector3 offset_pos = Vector3.Zero;
            foreach (CollisionShape col_shape in collision_spheres)
            {
                Vector3 test_vec = col_shape.GlobalTransform.AffineInverse().Xform(GlobalTransform.origin);
                Vector3 direction = test_vec - test_vec.Normalized() * (col_shape.Shape as SphereShape).Radius;
                float distance = test_vec.Length() - (col_shape.Shape as SphereShape).Radius;
                if (distance < 0)
                {
                    // offset_pos += col_shape.ToGlobal(direction) - col_shape.GlobalTransform.origin;
                    GlobalPosition -= col_shape.ToGlobal(direction) - col_shape.GlobalTransform.origin;
                }
            }
            // GlobalPosition -= offset_pos;
            // Vector3 tar_pos = GlobalPosition - offset_pos;
            // GlobalPosition = new Vector3(Mathf.Lerp(GlobalPosition.x, tar_pos.x, delta),
            //     Mathf.Lerp(GlobalPosition.y, tar_pos.y, delta),
            //     Mathf.Lerp(GlobalPosition.z, tar_pos.z, delta)
            //     );

        }


        // ############## Rotate the bone to point to this object #############
        Vector3 diff_vec_local = bone_transf_world.AffineInverse().Xform(GlobalTransform.origin).Normalized();
        Vector3 bone_forward_local = get_bone_forward_local();

        // # The axis+angle to rotate on, in local-to-bone space
        Vector3 bone_rotate_axis = bone_forward_local.Cross(diff_vec_local);
        float bone_rotate_angle = (float)Math.Acos(bone_forward_local.Dot(diff_vec_local));

        if (bone_rotate_axis.Length() < 1e-3)
        {
            return;
        }
        else if (float.IsNaN(bone_rotate_axis.x))
        {
            bone_rotate_axis = Vector3.Zero;
        }
        bone_rotate_axis = bone_rotate_axis.Normalized();

        // # Bring the axis to object space, WITHOUT translation (so only the BASIS is used) since vectors shouldn't be translated
        Vector3 bone_rotate_axis_obj = bone_transf_obj.basis.Xform(bone_rotate_axis).Normalized();
        if (float.IsNaN(bone_rotate_axis_obj.x) && bone_rotate_axis_obj.Length() < 1e-3)
        {
            return;
        }
        // Transform bone_new_transf_obj = new Transform(bone_transf_obj.basis.Rotated(bone_rotate_axis_obj, bone_rotate_angle), bone_transf_obj.origin);
        Transform bone_new_transf_obj = new Transform(bone_transf_obj.basis.Rotated(bone_rotate_axis_obj, bone_rotate_angle), bone_transf_rest_obj.origin);
        if (float.IsNaN(bone_new_transf_obj[0][0]))
        {
            bone_new_transf_obj = new Transform(); //# Corrupted somehow
        }

        skeleton.SetBoneGlobalPoseOverride(bone_id, bone_new_transf_obj, apply_percent/ 100.0f, true);
        // # Orient this object to the jigglebone
        Basis tar = (skeleton.GlobalTransform * skeleton.GetBoneGlobalPose(bone_id)).basis;
        SetGlobalTransform(new Transform(tar, GlobalTransform.origin));


    }

    public override void _PhysicsProcess(float delta)
    {
        frame++;
        if (frame > skip_frame)
        {
            frame = 0;
            float factor = (float)Convert.ToDecimal(1 + skip_frame);
            CallDeferred("JiggleBoneProcess", delta * factor);
        }
    }

    public void add_collision_shape(CollisionShape col_shape){
        if (col_shape is null)
        {
            return;
        }
        if (!(col_shape.Shape is SphereShape))
        {
            return;
        }
        collision_spheres.Add(col_shape);
    }
}
