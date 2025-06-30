# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2023 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban, Aaron Young
# =============================================================================
#
# Demo for the URDF -> Chrono parser in python
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.parsers as parsers

import time
import numpy as np

import sys, os
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)


def main():
    # Create a Chrono system
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


    # Create the parser instance
    # filename = project_root + "/data/robot/go2_description/urdf/go2_description.urdf"
    filename = project_root + "/data/robot/unitree_a1/urdf/a1sub.urdf"
    parser = parsers.ChParserURDF(filename)

    # Set root body pose
    parser.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5), chrono.QuatFromAngleZ(np.pi/2)))

    # Make all eligible joints as actuated
    parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION);

    parser.EnableCollisionVisualization()
    # parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_FORCE);


    # # Display raw XML string
    # print("\nURDF input\n")
    # print(parser.GetXMLstring())

    # # Report parsed elements
    # parser.PrintModelBodyTree()
    # parser.PrintModelBodies()
    # parser.PrintModelJoints()

    # Create the Chrono model
    parser.PopulateSystem(system)


    # Fix root body
    parser.GetRootChBody().SetFixed(False)
    

    # create contact material
    mat = chrono.ChContactMaterialSMC()
    mat.SetRestitution(0.0)
    mat.SetGn(60.0)
    mat.SetKn(2e5)

    # Create a floor body
    floor = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True, mat)
    floor.SetPos(chrono.ChVector3d(0, 0, 0.0))
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 10, 10)
    floor.SetFixed(True)
    system.AddBody(floor)
    
    # set collision for foot bodies
    foot_FR = parser.GetChBody("FR_foot")
    foot_FL = parser.GetChBody("FL_foot")
    foot_RR = parser.GetChBody("RR_foot")
    foot_RL = parser.GetChBody("RL_foot")
    
    foot_FR.EnableCollision(True)
    foot_FL.EnableCollision(True)
    foot_RR.EnableCollision(True)
    foot_RL.EnableCollision(True)

    foot_FR.GetCollisionModel().SetAllShapesMaterial(mat)
    foot_FL.GetCollisionModel().SetAllShapesMaterial(mat)
    foot_RR.GetCollisionModel().SetAllShapesMaterial(mat)
    foot_RL.GetCollisionModel().SetAllShapesMaterial(mat)
    
    # set motor list
    # motor_name_list = ["RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"]
    thigh_joints = ["RR_thigh_joint", "RL_thigh_joint", "FR_thigh_joint", "FL_thigh_joint"]
    calf_joints = ["RR_calf_joint", "RL_calf_joint", "FR_calf_joint", "FL_calf_joint"]
    hip_joints = ["RR_hip_joint", "RL_hip_joint", "FR_hip_joint", "FL_hip_joint"]

    motors_thigh = []
    motors_calf = []
    motors_hip = []
    for motor_name in thigh_joints:
        motors_thigh.append(parser.GetChMotor(motor_name))
    for motor_name in calf_joints:
        motors_calf.append(parser.GetChMotor(motor_name))
    for motor_name in hip_joints:
        motors_hip.append(parser.GetChMotor(motor_name))

    # try constant function
    cons_func_thigh = chrono.ChFunctionConst(-1.0)
    cons_func_calf = chrono.ChFunctionConst(1.5)
    cons_func_hip = chrono.ChFunctionConst(0.0)
    for motor in motors_thigh:
        motor.SetMotorFunction(cons_func_thigh)
    for motor in motors_calf:
        motor.SetMotorFunction(cons_func_calf)
    for motor in motors_hip:
        motor.SetMotorFunction(cons_func_hip)

    vis_enabled = True

    if vis_enabled:
        # Create the irrlicht visualization
        vis = irr.ChVisualSystemIrrlicht()
        vis.AttachSystem(system)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis.SetWindowSize(1024, 768)
        vis.SetWindowTitle("URDF Parser Demo")
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(1, 1, 2))
        vis.AddTypicalLights()

    # Simulation loop
    step_size = 1e-3
    render_step_size = 1/50
    sim_step = 0
    render_steps = int(render_step_size / step_size)
    control_step_size = 1/50
    control_steps = int(control_step_size / step_size)

    start_time = time.time()
    end_sim_time = 20

    # while (system.GetChTime() < end_sim_time):
    while vis.Run():
        sim_time = system.GetChTime()

        if (sim_step % render_steps == 0) and vis_enabled:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        if (sim_step % control_steps == 0):
            for motor in motors_thigh:
                motor.SetMotorFunction(cons_func_thigh)
            for motor in motors_calf:
                motor.SetMotorFunction(cons_func_calf)
            
            base_pos = parser.GetChBody("base").GetPos()
            print(f"base_pos: {base_pos}")
            base_heading = parser.GetChBody("base").GetRot().GetCardanAnglesZYX().z
            print(f"base_heading: {base_heading}")
            
            base_rot = parser.GetChBody("base").GetRot()
            # Create gravity vector in world frame (0,0,-9.81)
            gravity_world = chrono.ChVector3d(0, 0, -9.81)
            # Transform gravity to base frame
            gravity_base_projected = base_rot.RotateBack(gravity_world)
            # Normalize to get direction
            gravity_base_projected.Normalize()
            print(f"Gravity in base frame: {gravity_base_projected}")

            body_vel = parser.GetChBody("base").GetLinVel()
            print(f"body_vel: {body_vel}")

            body_ang_vel = parser.GetChBody("base").GetAngVelLocal()
            print(f"body_ang_vel: {body_ang_vel}")

        system.DoStepDynamics(step_size)
        sim_step += 1
    else:
        real_time = time.time() - start_time
        print(f"Simulation time: {sim_time}")
        print(f"Real time: {real_time}")
        print(f"RTF: {real_time / sim_time}")

if __name__ == "__main__":
    main()