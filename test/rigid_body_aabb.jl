using PhyBullet
using PyCall

client = @pycall pb.connect(pb.DIRECT)::Int64

sim = BulletSim(;client=client)

obj_dims = [1, 1, 1]
obj_col_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=obj_dims/2, physicsClientId=client)
obj_obj_id = pb.createMultiBody(baseCollisionShapeIndex=obj_col_id, basePosition=[0,0,0], physicsClientId=client)
pb.changeDynamics(obj_obj_id, -1; mass=1.0, restitution=0.9, lateralFriction=0, physicsClientId=client)

rigid_body = RigidBody(obj_obj_id)

state = BulletState(sim, [rigid_body])

display(state.kinematics[1].aabb)