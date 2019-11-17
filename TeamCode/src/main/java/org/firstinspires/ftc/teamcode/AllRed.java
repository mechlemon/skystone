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


@Autonomous(name = "AllRed", group = "red")

public class AllRed extends LinearOpMode {

    public enum Status {
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
        FORWARD2FOUNDATION2,
        DROPSTONE,
        LIFTARM,
        BACK2,
        MOVEFOUNDATION, // joystick both in
        RELEASEFOUNDATION,
        PUSHFOUNDATION,
        PARK,

        DONE,
    }

    Hardware hardware;

    @Override
    public void runOpMode() {
        hardware = new Hardware(hardwareMap, telemetry);

        double fordist = 56;

        waitForStart();

        Status status = Status.FORWARD2SCAN;
        Timer timer = new Timer();

        resetDrive(hardware, timer);

        long lasttime = System.currentTimeMillis();

        while (!isStopRequested()) {

            if (status == Status.FORWARD2SCAN) {
                if (13.5 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.drivetrain.forward(0.4);
                    hardware.dropStone();
                } else {
                    status = Status.SCAN1;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.SCAN1) {
                fordist = 50;
                if (timer.getElapsed() > 1) {
                    if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                        double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1) + 10;
                        hardware.drivetrain.right(0.004 * skystoneX + Math.copySign(0.15, skystoneX));
                        if (4 > Math.abs(skystoneX)) {
                            hardware.drivetrain.setPowers(0, 0, 0);
                            hardware.resetEncoders();
                            timer.reset();
                            status = Status.FORWARD2GRAB;
                        }
                    } else if (timer.getElapsed() > 2) {
                        status = Status.SCAN2;
                        resetDrive(hardware, timer);
                    }
                }
            }

            else if (status == Status.SCAN2) {
                fordist = 59;
                if (timer.getElapsed() < 1.2) {
                    hardware.steadyTranslationPIDF(0.40, 0);
                } else {
                    if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                        double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1);
                        hardware.drivetrain.right(0.004 * skystoneX + Math.copySign(0.15, skystoneX));
                        if (6 > Math.abs(skystoneX)) {
                            status = Status.FORWARD2GRAB;
                            resetDrive(hardware, timer);
                        }
                    } else if (timer.getElapsed() < 2) {
                        hardware.drivetrain.setPowers(0, 0, 0);
                    } else {
                        status = Status.SCAN3;
                        resetDrive(hardware, timer);
                    }
                }
            }

            else if (status == Status.SCAN3) {
                fordist = 68;
                if (timer.getElapsed() < 1.2) {
                    hardware.steadyTranslationPIDF(0.40, 0);
                }
                else if(timer.getElapsed() < 2){
                    hardware.steadyTranslationPIDF(0,0);
                } else {
                    if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                        double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1);
                        hardware.drivetrain.right(0.004 * skystoneX + Math.copySign(0.15, skystoneX));
                        if (6 > Math.abs(skystoneX)) {
                            hardware.drivetrain.setPowers(0, 0, 0);
                            hardware.resetEncoders();
                            timer.reset();
                            status = Status.FORWARD2GRAB;
                        }
                    } else if (timer.getElapsed() > 3) {
                        status = Status.FORWARD2GRAB;
                        resetDrive(hardware, timer);
                        hardware.vuforiaPhone.disable();
                    }

                }

            }

            else if (status == Status.FORWARD2GRAB) {
                if (15 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.steadyTranslationPIDF(0, 0.7);
                    hardware.funnelling();
                } else {
                    status = Status.GRABSTONE;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.GRABSTONE) {
                hardware.clampStone();
                if (1 < timer.getElapsed()) {
                    status = Status.LIFTSTONE;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.LIFTSTONE) {
                if (-40 > hardware.getArmAngle()) {
                    hardware.armApplyAntigrav(0.4);
                } else {
                    hardware.armApplyAntigrav(0);
                    status = Status.BACK1;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.BACK1) {
                if (-10 < Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.drivetrain.back(0.7);
                } else {
                    status = Status.TURN2FOUNDATION1;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.TURN2FOUNDATION1) {
                if (timer.getElapsed() < 1.2) {
                    hardware.steadyTranslationPIDF(0,0,-90);
                } else {
                    status = Status.FORWARD2FOUNDATION1;
                    hardware.clampFoundation();
                    hardware.releaseFoundation();
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.FORWARD2FOUNDATION1) {
                if (timer.getElapsed() > 0.5) {
                    if (fordist > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                        hardware.drivetrain.forward(0.7);
                    } else {
                        status = Status.TURN2FOUNDATION2;
                        resetDrive(hardware, timer);
                    }
                }
            }

            else if (status == Status.TURN2FOUNDATION2) {
                if (timer.getElapsed() < 1.2) {
                    hardware.steadyTranslationPIDF(0,0,0);
                } else {
                    status = Status.FORWARD2FOUNDATION2;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.FORWARD2FOUNDATION2) {
                if (18 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.drivetrain.forward(0.7);
                } else {
                    status = Status.DROPSTONE;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.DROPSTONE) {
                if (-60 < hardware.getArmAngle() && timer.getElapsed() < 3) {
                    hardware.armApplyAntigrav(-0.3);
                    hardware.drivetrain.setPowers(0, 0, 0);
                } else {
                    hardware.dropStone();
                    hardware.armApplyAntigrav(0);
                    status = Status.LIFTARM;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.LIFTARM) {
                if (timer.getElapsed() < 0.5) {
                    hardware.elevator.setPower(1);
                    hardware.armApplyAntigrav(0.3);
                } else {
                    hardware.elevator.setPower(0);
                    hardware.armApplyAntigrav(0);
                    hardware.clampFoundation();
                    status = Status.BACK2;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.BACK2) {
                if (timer.getElapsed() > 0.4) {
                    hardware.drivetrain.setPowers(-1, -1, 0);
                    if (3 < timer.getElapsed()) {
                        status = Status.MOVEFOUNDATION;
                        resetDrive(hardware, timer);
                    }
                }

            }

            else if (status == Status.MOVEFOUNDATION) {
                hardware.drivetrain.setPowers(1, -1, 1); //dependent
                if (1.2 < timer.getElapsed()) {
                    status = Status.RELEASEFOUNDATION;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.RELEASEFOUNDATION) {
                hardware.releaseFoundation();
                status = Status.PUSHFOUNDATION;
                resetDrive(hardware, timer);
            }

            else if (status == Status.PUSHFOUNDATION) {
                if (timer.getElapsed() < 2) {
                    hardware.drivetrain.forward(1);
                } else {
                    status = Status.PARK;
                    resetDrive(hardware, timer);
                }
            }

            else if (status == Status.PARK) {
                if (-30 < Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                    hardware.drivetrain.back(1);
                    hardware.elevator.setPower(-0.7);
                    hardware.arm.setPower(-0.1);
                } else {
                    hardware.elevator.setPower(0);
                    hardware.arm.setPower(0);
                    resetDrive(hardware, timer);
                    status = Status.DONE;
                }
            }

            telemetry.addData("dt", System.currentTimeMillis() - lasttime);
            lasttime = System.currentTimeMillis();

            hardware.drivetrain.execute();
            telemetry.addData("Status", status);
//            telemetry.addData("heading", hardware.imu.getHeading());
//            telemetry.addData("driveL", hardware.getLeftDrivePos());
//            telemetry.addData("driveR", hardware.getRightDrivePos());
//            telemetry.addData("driveStrafe", hardware.getStrafeDrive1Pos());
//            telemetry.addData("arm", hardware.arm.getCurrentPosition());
//            telemetry.addData("grabL", hardware.grabLeft.getPosition());
//            telemetry.addData("grabR", hardware.grabRight.getPosition());
//            telemetry.addData("grabFoundationL", hardware.grabFoundationLeft.getPosition());
//            telemetry.addData("grabFoundationR", hardware.grabFoundationRight.getPosition());
            telemetry.update();

        }
    }

    void resetDrive(Hardware hardware, Timer timer) {
        hardware.drivetrain.setPowers(0, 0, 0);
        hardware.resetEncoders();
        timer.reset();
    }

}
