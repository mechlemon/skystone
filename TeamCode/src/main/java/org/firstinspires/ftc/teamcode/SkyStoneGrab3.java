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
        SCANNING,
        FORWARD2GRAB,
        GRABSTONE,
        LIFTSTONE,
        BACK2WALL1,
        RIGHT2FOUNDATION,
        BACK2WALL2,
        FORWARD2FOUNDATION,
        GRABFOUNDATION,
        BACK2WALL3,
        RELEASEFOUNDATION,
        PARK,


        DONE,

    }

    Hardware hardware;

    @Override public void runOpMode() {
        hardware = new Hardware(hardwareMap, telemetry);
        hardware.imu.setHeadingAxis(IMU.HeadingAxis.ROLL);

        waitForStart();

        Status status = Status.FORWARD2SCAN;
        Timer timer = new Timer();
        hardware.resetEncoders();

        while(!isStopRequested()){

             if(status == Status.FORWARD2SCAN) {
                 if (1100 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                     hardware.drivetrain.forward(0.4);
                     hardware.dropStone();
                 } else {
                     hardware.drivetrain.setPowers(0,0,0);
                     status = Status.SCANNING;
                     timer.reset();
                 }
             }

            if(status == Status.SCANNING) {
                if(1 < timer.getElapsed()) {
                    if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                        double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1) - 40;
                        hardware.drivetrain.right(0.01 * skystoneX + Math.copySign(0.15, skystoneX));
                        if (5 > Math.abs(skystoneX)) {
                            hardware.drivetrain.setPowers(0,0,0);
                            hardware.resetEncoders();
                            status = Status.FORWARD2GRAB;
                        }
                        telemetry.addData("targetPos", skystoneX);
                    } else {
                        hardware.drivetrain.left(0.18);
                    }
                }else{
                    hardware.drivetrain.setPowers(0,0,0);
                }
            }

            if(status == Status.FORWARD2GRAB) {
                if (850 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.steadyForward(0.35);
                } else {
                    hardware.drivetrain.setPowers(0,0,0);
                    status = Status.GRABSTONE;
                    timer.reset();
                }
            }

            if(status == Status.GRABSTONE) {
                hardware.clampStone();
                hardware.drivetrain.setPowers(0,0,0);
                if(2 < timer.getElapsed()){
                    status = Status.LIFTSTONE;
                }
            }

            if(status == Status.LIFTSTONE) {
                if (360 > hardware.arm.getCurrentPosition()) {
                    hardware.armApplyAntigrav(0.4);
                    hardware.drivetrain.setPowers(0,0,0);
                } else {
                    hardware.armApplyAntigrav(0);
                    hardware.drivetrain.setPowers(0,0,0);
                    status = Status.BACK2WALL1;
                    timer.reset();
                }
            }

            if(status == Status.BACK2WALL1) {
                hardware.drivetrain.back(0.7);
                if(1 < timer.getElapsed()){
                    hardware.drivetrain.setPowers(0,0,0);
                    timer.reset();
                    status = Status.RIGHT2FOUNDATION;
                }
            }

            if(status == Status.RIGHT2FOUNDATION) {
                hardware.steadyLeft(-1);
                if(2 < timer.getElapsed()){
                    hardware.drivetrain.setPowers(0,0,0);
                    hardware.resetEncoders();
                    timer.reset();
                    status = Status.BACK2WALL2;
                }
            }

            if(status == Status.BACK2WALL2) {
                hardware.drivetrain.back(0.7);
                if(0.5 < timer.getElapsed()){
                    hardware.drivetrain.setPowers(0,0,0);
                    timer.reset();
                    status = Status.FORWARD2FOUNDATION;
                }
            }

            if(status == Status.FORWARD2FOUNDATION) {
                if(200 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.steadyForward(0.3);
                } else{
                    hardware.drivetrain.setPowers(0,0,0);
                    status = Status.GRABFOUNDATION;
                }
            }

            if(status == Status.GRABFOUNDATION){
                hardware.clampFoundation();
                hardware.dropStone();
                timer.reset();
                status = Status.BACK2WALL3;
            }

            if(status == Status.BACK2WALL3){
                hardware.drivetrain.back(0.7);
                if(1 < timer.getElapsed()){
                    hardware.drivetrain.setPowers(0,0,0);
                    timer.reset();
                    status = Status.RELEASEFOUNDATION;
                }
            }

            if(status == Status.RELEASEFOUNDATION){
                hardware.releaseFoundation();
                timer.reset();
                status = Status.PARK;
            }

            if(status == Status.PARK){
                hardware.steadyLeft(0.7);
                if(1.5 < timer.getElapsed()){
                    hardware.drivetrain.setPowers(0,0,0);
                    timer.reset();
                    status = Status.DONE;
                }
            }


            if(status == Status.DONE) {
                hardware.resetMotors();
            }


            hardware.drivetrain.execute();
            telemetry.addData("Status", status);
            telemetry.addData("TimerElapsed", timer.getElapsed());
            telemetry.addData("heading", hardware.imu.getHeading());
            telemetry.addData("driveL", hardware.drivetrain.leftMotor.getCurrentPosition());
            telemetry.addData("driveR", hardware.drivetrain.rightMotor.getCurrentPosition());
            telemetry.addData("driveStrafe", hardware.drivetrain.strafeMotor1.getCurrentPosition());
            telemetry.addData("arm", hardware.arm.getCurrentPosition());
            telemetry.addData("grabL", hardware.grabLeft.getPosition());
            telemetry.addData("grabR", hardware.grabRight.getPosition());
            telemetry.addData("grabFoundationL", hardware.grabFoundationLeft.getPosition());
            telemetry.addData("grabFoundationR", hardware.grabFoundationRight.getPosition());
            telemetry.update();

        }


    }


}
