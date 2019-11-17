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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="SteadyTest", group ="concept")

public class SteadyTest extends OpMode {

    Hardware hardware;

    long lasttime;


//    String[] titles = new String[] {"p", "i", "d", "b", "f", "t", "vt"}; //names of the tuner values
////    double[] values = new double[] {0.002, 6.77e-5, -0.0064, 0.229, 0.10, 5, 0.1}; //default tuner values
//    double[] values = new double[] {0.002, 0, 1e-5, 0.8, 0.10, 5, 0.1}; //default tuner values
//    Tuner tuner;



    public void init(){
        hardware = new Hardware(hardwareMap, telemetry, true);
        lasttime = System.currentTimeMillis();
    }

    boolean running = false;


    public void loop() {
        if(gamepad1.x){
            running = true;
        }else if(gamepad1.y){
            running = false;
        }

        if(running){
            hardware.steadyTranslationPIDF(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
        }else{
            hardware.drivetrain.setPowers(0,0,0);
        }

        telemetry.addData("dt", System.currentTimeMillis() - lasttime);
        lasttime = System.currentTimeMillis();

        hardware.drivetrain.execute();
        telemetry.addData("running", running);
        telemetry.addData("P", hardware.steadyPIDF.getPID()[0]);
        telemetry.addData("I", hardware.steadyPIDF.getPID()[1]);
        telemetry.addData("D", hardware.steadyPIDF.getPID()[2]);
        telemetry.addData("F", hardware.steadyPIDF.getPID()[3]);
        telemetry.addData("vel", hardware.steadyPIDF.getVel());
        telemetry.update();

    }

}
