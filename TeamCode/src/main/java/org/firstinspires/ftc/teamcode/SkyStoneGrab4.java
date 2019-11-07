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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name="SkyStoneGrabRed4", group ="comp")

public class SkyStoneGrab4 extends OpMode {

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
    Status status;
    Timer timer;

    Calculate.PIDF stonePID = new Calculate.PIDF(0.02, 0, 0, 0.1, 8);

    @Override public void init(){
        hardware = new Hardware(hardwareMap, telemetry);
        status = Status.FORWARD2SCAN;
        timer = new Timer();
        hardware.resetEncoders();
    }

    @Override public void loop() {
         if(status == Status.FORWARD2SCAN) {
             if (14 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                 hardware.drivetrain.forward(0.4);
                 hardware.dropStone();
             } else {
                 hardware.drivetrain.setPowers(0,0,0);
                 status = Status.SCANNING;
                 timer.reset();
             }
         }

        else if(status == Status.SCANNING) {
            if(1 < timer.getElapsed()) {
                if (hardware.vuforiaPhone.getSkystoneTranslation() != null) {
                    double skystoneX = hardware.vuforiaPhone.getSkystoneTranslation().get(1);
                    stonePID.loop(skystoneX, 30);
                    if(stonePID.inTolerance()){
                        hardware.drivetrain.setPowers(0,0,0);
                        hardware.resetEncoders();
                        status = Status.FORWARD2GRAB;
                    }else{
                        hardware.drivetrain.right(stonePID.getPower());
                    }
                } else {
                    hardware.drivetrain.left(0.2);
                }
            }else{
                hardware.drivetrain.setPowers(0,0,0);
            }
        }

        else if(status == Status.FORWARD2GRAB) {
            if (8 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
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
            if (-30 > hardware.getArmAngle()) {
                hardware.armApplyAntigrav(0.4);
                hardware.drivetrain.setPowers(0,0,0);
            } else {
                hardware.armApplyAntigrav(0);
                hardware.drivetrain.setPowers(0,0,0);
                status = Status.BACK2WALL1;
                timer.reset();
            }
        }

        else if(status == Status.BACK2WALL1) {
            hardware.drivetrain.back(0.7);
            if(1.5 < timer.getElapsed()){
                hardware.drivetrain.setPowers(0,0,0);
                timer.reset();
                hardware.imu.resetHeading();
                status = Status.RIGHT2FOUNDATION;
            }
        }

        else if(status == Status.RIGHT2FOUNDATION) {
            hardware.steadyTranslation(-1, 0);
            if(5 < timer.getElapsed()){
                hardware.drivetrain.setPowers(0,0,0);
                hardware.resetEncoders();
                timer.reset();
                status = Status.BACK2WALL2;
            }
        }

        else if(status == Status.BACK2WALL2) {
            hardware.drivetrain.back(0.7);
            if(0.5 < timer.getElapsed()){
                hardware.drivetrain.setPowers(0,0,0);
                timer.reset();
                status = Status.FORWARD2FOUNDATION;
            }
        }

        else if(status == Status.FORWARD2FOUNDATION) {
            if(5 > Calculate.average(hardware.getLeftDrivePos(), hardware.getRightDrivePos())) {
                hardware.drivetrain.forward(0.3);
            } else{
                hardware.drivetrain.setPowers(0,0,0);
                status = Status.GRABFOUNDATION;
            }
        }

         else if(status == Status.GRABFOUNDATION){
            hardware.clampFoundation();
            hardware.dropStone();
            timer.reset();
            status = Status.BACK2WALL3;
        }

         else if(status == Status.BACK2WALL3){
            hardware.drivetrain.back(0.7);
            if(2 < timer.getElapsed()){
                hardware.drivetrain.setPowers(0,0,0);
                timer.reset();
                status = Status.RELEASEFOUNDATION;
            }
        }

         else if(status == Status.RELEASEFOUNDATION){
            hardware.releaseFoundation();
            timer.reset();
            status = Status.PARK;
        }

         else if(status == Status.PARK){
            hardware.steadyTranslation(0.7, 0);
            if(1.5 < timer.getElapsed()){
                hardware.drivetrain.setPowers(0,0,0);
                timer.reset();
                status = Status.DONE;
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
