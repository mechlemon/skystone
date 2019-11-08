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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TesterIterative", group ="concept")

public class TesterIterative extends OpMode {

    Hardware hardware;

    int turn = 0;

    String[] titles = new String[] {"kP", "kI", "kD", "kF", "tolerance"}; //names of the tuner values
    double[] values = new double[] {(1/90.0), 0.001, 0.01, 0.1, 5}; //default tuner values
    Tuner tuner;

    Calculate.PIDF pid = new Calculate.PIDF(0,0,0,0,0);


    public void init(){
        hardware = new Hardware(hardwareMap, telemetry, false);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    public void loop() {

        tuner.tune();
        pid.setConstants(tuner.get("kP"),
                         tuner.get("kI"),
                         tuner.get("kD"),
                         tuner.get("kF"),
                         tuner.get("tolerance"));

        if(gamepad1.x){
            hardware.imu.resetHeading();
        }
        if(gamepad1.dpad_left){
            turn = 1;
        }
        if(gamepad1.dpad_right){
            turn = -1;
        }

        if(turn == 1){
            pid.loop(hardware.imu.getHeading(), 90);
            hardware.drivetrain.setPowers(-pid.getPower(), pid.getPower(), 0);
        }

        if(turn == -1){
            pid.loop(hardware.imu.getHeading(), -90);
            hardware.drivetrain.setPowers(pid.getPower(), -pid.getPower(), 0);        }

        if(hardware.steadyPIDF.inTolerance()){
            turn = 0;
        }

        if (turn == 0) {
            hardware.drivetrain.setPowers(0,0,0);
        }


        hardware.drivetrain.execute();
        telemetry.addData("turn", turn);
        telemetry.addData("heading", hardware.imu.getHeading());
        telemetry.addData("Ldrive", hardware.getLeftDrivePos());
        telemetry.addData("Rdrive", hardware.getRightDrivePos());
        telemetry.addData("armpos", hardware.getArmAngle());
        telemetry.update();

    }

}
