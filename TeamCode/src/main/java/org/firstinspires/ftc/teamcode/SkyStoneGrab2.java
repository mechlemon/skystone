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


@Autonomous(name="SkyStoneGrabRed2", group ="comp")

public class SkyStoneGrab2 extends LinearOpMode {

    public enum Status{
        FORWARD2SCAN,
        SCANNING,
        FORWARD2GRAB,
        GRABSTONE,
        LIFTSTONE,
        BACK2WALL,
        RIGHT2FOUNDATION,
        FORWARD2FOUNDATION,
        GRABFOUNDATION,


        DONE,

    }

    Hardware hardware;
    Status status;
    Timer timer = new Timer();

    @Override public void runOpMode() {
        hardware = new Hardware(hardwareMap, telemetry);
        hardware.imu.setHeadingAxis(IMU.HeadingAxis.ROLL);

        waitForStart();

        status = Status.FORWARD2SCAN;

        while(!isStopRequested()){

            switch (status) {

                case FORWARD2SCAN:
                    if (1100 > Calculate.average(hardware.drivetrain.leftMotor.getCurrentPosition(), hardware.drivetrain.rightMotor.getCurrentPosition())) {
                        hardware.drivetrain.forward(0.4);
                    } else {
                        hardware.drivetrain.clear();
                        timer.set(0.5);
                        status = Status.SCANNING;
                    }
                    break;

                case SCANNING:
                    if(!timer.isDone()) break;
                    if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                        double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1) - 40;
                        hardware.drivetrain.right(0.01 * skystoneX + Math.copySign(0.15,skystoneX));
                        if (10 > Math.abs(skystoneX)) {
                            hardware.drivetrain.clear();
                            status = Status.FORWARD2GRAB;
                        }
                        telemetry.addData("targetPos", skystoneX);

                    } else {
                        hardware.drivetrain.left(0.22);
                    }
                    break;

                case FORWARD2GRAB:
                    if (1600 > Calculate.average(hardware.drivetrain.leftMotor.getCurrentPosition(), hardware.drivetrain.rightMotor.getCurrentPosition())) {
                        hardware.drivetrain.forward(0.35);
                    } else {
                        hardware.drivetrain.clear();
                        status = Status.GRABSTONE;
                    }
                    break;

                case GRABSTONE:
                    hardware.clampStone(1);
                    timer.set(1);
                    status = Status.LIFTSTONE;
                    break;

                case LIFTSTONE:
                    if(!timer.isDone()) break;
                    if(400 > hardware.arm.getCurrentPosition()) {
                        hardware.arm.setPower(0.3);
                    } else {
                        hardware.arm.setPower(0);
                        hardware.drivetrain.back(0.4);
                        timer.set(2);
                        status = Status.BACK2WALL;
                    }
                    break;

                case BACK2WALL:
                    if(!timer.isDone()) break;
                    hardware.drivetrain.right(1);
                    timer.set(3);
                    status = Status.RIGHT2FOUNDATION;
                    break;

                case RIGHT2FOUNDATION:
                    if(!timer.isDone()) break;
                    status = Status.DONE;
                    break;



                case DONE:
                    hardware.resetMotors();
                    break;


            }

            hardware.drivetrain.execute();
            telemetry.addData("Status", status);
            telemetry.addData("timerElapsed", timer.getElapsed());
//            telemetry.addData("heading", hardware.imu.getHeading());
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
