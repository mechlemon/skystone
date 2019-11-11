/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="FoundationGrabRed", group ="comp")

public class FoundationGrabRed extends LinearOpMode {

    public enum Status{
        WAIT,
        FORWARD2FOUNDATION2,
        DROPSTONE,
        LIFTARM,
        BACK2,
        MOVEFOUNDATION, // joystick both in
        RELEASEFOUNDATION,
        PARK,


        DONE,

    }

    Hardware hardware;

    @Override public void runOpMode() {
        hardware = new Hardware(hardwareMap, telemetry);

        double fordist = 60;



        waitForStart();


        Status status = Status.WAIT;
        Timer timer = new Timer();
        hardware.resetEncoders();

        while(!isStopRequested()){

            if((timer.getElapsed() > 0) && (status == Status.WAIT)){
                status = Status.FORWARD2FOUNDATION2;
            }

            if(status == Status.FORWARD2FOUNDATION2) {
                if (30 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.drivetrain.forward(0.7);
                } else {
                    hardware.drivetrain.setPowers(0,0,0);
                    hardware.resetEncoders();
                    status = Status.LIFTARM;
                    timer.reset();
                    hardware.imu.resetHeading();
                }
            }


            else if(status == Status.LIFTARM){
                if (timer.getElapsed() < 1) {
                    hardware.armApplyAntigrav(0.3);
                } else {
                    hardware.armApplyAntigrav(0);
                    hardware.drivetrain.setPowers(0,0,0);
                    hardware.resetEncoders();
                    hardware.clampFoundation();
                    status = Status.BACK2;
                    timer.reset();
                }
            }

            else if(status == Status.BACK2){
                if(timer.getElapsed() > 1){
                    hardware.drivetrain.setPowers(-1, -1,0);
                    if(3 < timer.getElapsed()){
                        hardware.drivetrain.setPowers(0,0,0);
                        hardware.resetEncoders();
                        timer.reset();
                        status = Status.MOVEFOUNDATION;
                    }
                }

            }

            else if(status == Status.MOVEFOUNDATION){
                hardware.drivetrain.setPowers(1, -1,1); //dependent
                if(1.2 < timer.getElapsed()){
                    hardware.drivetrain.setPowers(0,0,0);
                    hardware.resetEncoders();
                    timer.reset();
                    status = Status.RELEASEFOUNDATION;
                }
            }

            else if(status == Status.RELEASEFOUNDATION){
                hardware.releaseFoundation();
                timer.reset();
                status = Status.PARK;
                hardware.resetEncoders();
            }

            else if(status == Status.PARK) {
                if (timer.getElapsed() < 2) {
                    hardware.drivetrain.forward(1);
                } else {
                    hardware.drivetrain.setPowers(0,0,0);
                    hardware.resetEncoders();
                    status = Status.DONE;
                    timer.reset();
                    hardware.imu.resetHeading();
                }
            }

            if(status == Status.DONE){
                if(-30 < Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())){
                    hardware.drivetrain.back(1);
                    hardware.elevator.setPower(-0.7);
                    hardware.arm.setPower(-0.1);
                }else{
                    hardware.drivetrain.setPowers(0,0,0);
                    hardware.elevator.setPower(0);
                    hardware.arm.setPower(0);


                }
            }


            hardware.drivetrain.execute();
            telemetry.addData("Status", status);
            telemetry.addData("TimerElapsed", timer.getElapsed());
            telemetry.addData("heading", hardware.imu.getHeading());
            telemetry.addData("driveL", hardware.getLeftDrivePos());
            telemetry.addData("driveR", hardware.getRightDrivePos());
            telemetry.addData("driveStrafe", hardware.getStrafeDrive1Pos());
            telemetry.addData("arm", hardware.arm.getCurrentPosition());
            telemetry.addData("grabL", hardware.grabLeft.getPosition());
            telemetry.addData("grabR", hardware.grabRight.getPosition());
            telemetry.addData("grabFoundationL", hardware.grabFoundationLeft.getPosition());
            telemetry.addData("grabFoundationR", hardware.grabFoundationRight.getPosition());
            telemetry.update();

        }


    }


}
