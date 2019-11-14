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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;



@TeleOp(name="TesterIterative", group ="concept")

public class TesterIterative extends OpMode {

    Hardware hardware;


    String[] titles = new String[] {"p", "i", "d", "b", "f", "t", "spinspeed"}; //names of the tuner values
//    double[] values = new double[] {0.01111111111111, 0.000001, 0.02929, 0.745, 0.1, 0}; //default tuner values
    double[] values = new double[] {0.0378, 0.000001, -0.05588, 0.745, 0.07, 0, 0.5}; //default tuner values
    Tuner tuner;



    public void init(){
        hardware = new Hardware(hardwareMap, telemetry, false);
        tuner = new Tuner(titles, values, gamepad1, telemetry);
    }

    boolean PIDF;
    double angle = 0;

    public void loop() {


        if(gamepad1.x){
            PIDF = true;
        }else if(gamepad1.y){
            PIDF = false;
        }

        angle += tuner.get("spinspeed") * 10 *gamepad1.right_stick_x;

        tuner.tune();

        if(PIDF){
            hardware.steadyPIDF.setConstants(tuner.get("p"),
                                             tuner.get("i"),
                                             tuner.get("d"),
                                             tuner.get("b"),
                                             tuner.get("f"),
                                             tuner.get("t"));
            hardware.steadyTranslationPIDF(-gamepad1.left_stick_x, -gamepad1.left_stick_y, angle);
        }else{
            hardware.drivetrain.clear();
        }



        hardware.drivetrain.execute();
        telemetry.addData("heading", hardware.imu.getHeading());
        telemetry.addData("angle", angle);
        telemetry.addData("PIDF", PIDF);
        telemetry.addData("P", hardware.steadyPIDF.getPID()[0]);
        telemetry.addData("I", hardware.steadyPIDF.getPID()[1]);
        telemetry.addData("D", hardware.steadyPIDF.getPID()[2]);
        telemetry.addData("F", hardware.steadyPIDF.getPID()[3]);
        telemetry.update();

    }

}
