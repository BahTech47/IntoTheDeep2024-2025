package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="SensorTeste", group="BahTech")

// Definição dos motores
public class SensorTeste extends LinearOpMode {

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    ColorSensor RS1;
    ColorSensor RS2;
    boolean amostra = false;

    // Definição de classes dos motores
    @Override
    public void runOpMode() {
        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        RS1 = hardwareMap.get(ColorSensor.class, "RS1");
        RS2 = hardwareMap.get(ColorSensor.class, "RS2");

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //Aqui o código está definindo as ações de acordo com o controle de PS4
        while (opModeIsActive()) {

            // Movimento para frente e para trás
            float powerFR = gamepad1.left_trigger - gamepad1.right_trigger;
            float powerBR = gamepad1.left_trigger - gamepad1.right_trigger;
            float powerFL = gamepad1.left_trigger - gamepad1.right_trigger;
            float powerBL = gamepad1.left_trigger - gamepad1.right_trigger;

            // Movimento de direita para esquerda
            float powerFR1 = + gamepad1.right_stick_x;
            float powerBR1 = - gamepad1.right_stick_x;
            float powerFL1 = - gamepad1.right_stick_x;
            float powerBL1 = + gamepad1.right_stick_x;

            // Movimento de giro
            float powerFR2 = + gamepad1.left_stick_x;
            float powerBR2 = + gamepad1.left_stick_x;
            float powerFL2 = - gamepad1.left_stick_x;
            float powerBL2 = - gamepad1.left_stick_x;

            // Aqui o código está definindo a velocidade que cada motor utilizara em seus movimentos
            FL.setPower(powerFL*0.4 + powerFL1*0.8 + powerFL2*0.4 );
            BL.setPower(powerBL*0.4 + powerBL1*0.8 + powerBL2*0.4);
            FR.setPower(powerFR*0.45 + powerFR1*0.85 + powerFR2*0.4);
            BR.setPower(powerBR*0.45 + powerBR1*0.85 + powerBR2*0.4);

            telemetry.addData("Red1", RS1.red());
            telemetry.addData("Green1", RS1.green());
            telemetry.addData("Blue1", RS1.blue());
            telemetry.addData("Red2", RS2.red());
            telemetry.addData("Green2", RS2.green());
            telemetry.addData("Blue2", RS2.blue());
            telemetry.addData("Distance1", "%.3f", ((DistanceSensor) RS1).getDistance(DistanceUnit.CM));
            telemetry.addData("Distance2", "%.3f", ((DistanceSensor) RS2).getDistance(DistanceUnit.CM));
            telemetry.addData("amostra", amostra);
            telemetry.update();

            if (((DistanceSensor) RS1).getDistance(DistanceUnit.CM) < 10 || ((DistanceSensor) RS2).getDistance(DistanceUnit.CM) < 10) {
                amostra = true;
            } else{
                amostra = false;
            }
        }
    }
}

// todo: write your code here