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


@Autonomous(name="SkyStoneGrabRed3", group ="comp")

public class SkyStoneGrab3 extends LinearOpMode {

    public enum Status{
        FORWARD2SCAN,
        SCAN1,
        SCAN2,
        SCAN3,
        FORWARD2GRAB,
        GRABSTONE,
        LIFTSTONE,
        BACK1,
        TURN2FOUNDATION1,
        FORWARD2FOUNDATION1,
        TURN2FOUNDATION2,
        ALIGN1,
        ALIGN2,
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


        Status status = Status.FORWARD2SCAN;
        Timer timer = new Timer();
        hardware.resetEncoders();

        while(!isStopRequested()){

             if(status == Status.FORWARD2SCAN) {
                 if (15 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                     hardware.drivetrain.forward(0.4);
                     hardware.dropStone();
                 } else {
                     hardware.drivetrain.setPowers(0,0,0);
                     status = Status.SCAN1;
                     timer.reset();
                 }
             }

            else if(status == Status.SCAN1) {
                 if(timer.getElapsed() > 1) {
                     hardware.drivetrain.setPowers(0, 0, 0);

                     if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                         double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1) + 10;
                         hardware.drivetrain.right(0.004 * skystoneX + Math.copySign(0.15, skystoneX));
                         if (6 > Math.abs(skystoneX)) {
                             hardware.drivetrain.setPowers(0,0,0);
                             hardware.resetEncoders();
                             timer.reset();
                             fordist = 50;
                             status = Status.FORWARD2GRAB;
                         }
                     }else if(timer.getElapsed() > 2.5) {
                         hardware.drivetrain.left(0.40);
                         status = Status.SCAN2;
                         hardware.resetEncoders();
                         timer.reset();
                     }

                 }
            }

             else if(status == Status.SCAN2) {
                 if(timer.getElapsed() > 1) {
                     hardware.drivetrain.setPowers(0, 0, 0);

                     if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                         double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1) + 10;
                         hardware.drivetrain.right(0.004 * skystoneX + Math.copySign(0.1, skystoneX));
                         if (6 > Math.abs(skystoneX)) {
                             hardware.drivetrain.setPowers(0,0,0);
                             hardware.resetEncoders();
                             timer.reset();
                             fordist = 53;
                             status = Status.FORWARD2GRAB;
                         }
                     }else if(timer.getElapsed() > 2.5) {
                         hardware.drivetrain.left(0.40);
                         status = Status.SCAN3;
                         hardware.resetEncoders();
                         timer.reset();
                     }

                 }
             }

             else if(status == Status.SCAN3) {
                 if(timer.getElapsed() > 1) {
                     hardware.drivetrain.setPowers(0, 0, 0);

                     if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                         double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1) + 10;
                         hardware.drivetrain.right(0.004 * skystoneX + Math.copySign(0.1, skystoneX));
                         if (6 > Math.abs(skystoneX)) {
                             hardware.drivetrain.setPowers(0,0,0);
                             hardware.resetEncoders();
                             timer.reset();
                             fordist = 58;
                             status = Status.FORWARD2GRAB;
                         }
                     }else if(timer.getElapsed() > 2.5) {
                             status = Status.FORWARD2GRAB;
                             hardware.resetEncoders();
                                fordist = 58;

                         timer.reset();

                     }

                 }

             }

            else if(status == Status.FORWARD2GRAB) {
                if (11 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.drivetrain.forward(0.35);
                } else {
                    hardware.drivetrain.setPowers(0,0,0);
                    status = Status.GRABSTONE;
                    timer.reset();
                }
            }

            else if(status == Status.GRABSTONE) {
                hardware.clampStone();
                hardware.drivetrain.setPowers(0,0,0);
                if(1 < timer.getElapsed()){
                    status = Status.LIFTSTONE;
                }
            }

            else if(status == Status.LIFTSTONE) {
                if (-40 > hardware.getArmAngle()) {
                    hardware.armApplyAntigrav(0.4);
                    hardware.drivetrain.setPowers(0,0,0);
                } else {
                    hardware.armApplyAntigrav(0);
                    hardware.drivetrain.setPowers(0,0,0);
                    hardware.resetEncoders();
                    status = Status.BACK1;
                    timer.reset();
                }
            }

            else if(status == Status.BACK1) {
                if (-10 < Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.drivetrain.forward(-0.7);
                } else {
                    hardware.drivetrain.setPowers(0,0,0);
                    status = Status.TURN2FOUNDATION1;
                    timer.reset();
                    hardware.imu.resetHeading();
                }
            }

            else if(status == Status.TURN2FOUNDATION1){
                if(-75 < hardware.imu.getHeading()){
                    hardware.drivetrain.setPowers(0.5, -0.5, 0);
                }else{
                    hardware.drivetrain.setPowers(0,0,0);
                    status = Status.FORWARD2FOUNDATION1;
                    hardware.resetEncoders();
                    hardware.clampFoundation();
                    hardware.releaseFoundation();
                    timer.reset();
                }
            }

            else if(status == Status.FORWARD2FOUNDATION1) {
                if(timer.getElapsed() > 0.5) {
                    if (fordist > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                        hardware.drivetrain.forward(0.7);
                    } else {
                        hardware.drivetrain.setPowers(0, 0, 0);
                        status = Status.TURN2FOUNDATION2;
                        timer.reset();
                        hardware.imu.resetHeading();
                    }
                }
            }

             else if(status == Status.TURN2FOUNDATION2){
                 if(50 > hardware.imu.getHeading()){
                     hardware.drivetrain.setPowers(-0.5, 0.5, 0);
                 }else{
                     hardware.drivetrain.setPowers(0,0,0);
                     status = Status.FORWARD2FOUNDATION2;
                     timer.reset();
                     hardware.resetEncoders();
                 }
             }



             else if(status == Status.FORWARD2FOUNDATION2) {
                 if (10 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                     hardware.drivetrain.forward(0.7);
                 } else {
                     hardware.drivetrain.setPowers(0,0,0);
                     hardware.resetEncoders();
                     status = Status.DROPSTONE;
                     timer.reset();
                     hardware.imu.resetHeading();
                 }
             }

             else if(status == Status.DROPSTONE) {
                 if (-40 < hardware.getArmAngle()) {
                     hardware.armApplyAntigrav(-0.3);
                     hardware.drivetrain.setPowers(0,0,0);
                 } else {
                     hardware.dropStone();
                     hardware.armApplyAntigrav(0);
                     hardware.resetEncoders();
                     status = Status.LIFTARM;
                     timer.reset();
                 }
             }

             else if(status == Status.LIFTARM){
                 if (timer.getElapsed() < 1) {
                     hardware.elevator.setPower(1);
                     hardware.armApplyAntigrav(0.3);
                 } else {
                     hardware.elevator.setPower(0);
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
                     if(2.5 < timer.getElapsed()){
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
                 if(timer.getElapsed() < 1.2){
                     hardware.drivetrain.back(1);
                 }else{
                     hardware.drivetrain.setPowers(0,0,0);
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
