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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name="SkyStoneGrabRed2", group ="comp")

public class SkyStoneGrab2 extends LinearOpMode {

    public enum Status{
        FORWARD2SCAN,
        SCANNING,
        FORWARD2GRAB,
        GRAB,
        BACK2WALL,
        LEFT2FOUNDATION,
        FORWARD2FOUNDATION,

        DONE,

    }

    Hardware hardware;
    Status status;

    @Override public void runOpMode() {
        hardware = new Hardware(hardwareMap, telemetry);
        hardware.imu.setHeadingAxis(IMU.HeadingAxis.ROLL);

        waitForStart();

        status = Status.FORWARD2SCAN;

        while(!isStopRequested()){
            hardware.clamp(1);
            hardware.clamp(0);
            hardware.clamp(1);
            hardware.clamp(0);
            try {
                switch (status) {

                    case FORWARD2SCAN:
                        if (1100 > Calculate.average(hardware.drivetrain.leftMotor.getCurrentPosition(), hardware.drivetrain.rightMotor.getCurrentPosition())) {
                            hardware.drivetrain.forward(0.4);
                        } else {
                            hardware.drivetrain.clear();
                            try{
                                wait(500);
                            }catch (Exception e){
                                telemetry.addData("error", e);
                            }
                            status = Status.SCANNING;
                        }
                        break;

                    case SCANNING:
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
                            status = Status.GRAB;
                        }
                        break;

                    case GRAB:
                        hardware.clamp(1);

                        try{
                            wait(1000);
                        }catch (Exception e){
                            telemetry.addData("error", e);
                        }

                        if (400 > hardware.arm.getCurrentPosition()) {
                            hardware.arm.setPower(0.3);
                        } else {
                            hardware.resetMotors();
                            status = Status.BACK2WALL;
                        }
                        break;


                    case BACK2WALL:
                        hardware.drivetrain.back(0.5);
                        break;

                    case DONE:
                        hardware.resetMotors();
                        break;


                }

            hardware.drivetrain.execute();
            telemetry.addData("Status", status);
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
            }catch(Exception e){
                telemetry.addData("error", e);
                telemetry.update();
            }
        }


    }

}
