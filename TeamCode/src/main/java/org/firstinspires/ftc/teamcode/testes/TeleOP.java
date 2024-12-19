package org.firstinspires.ftc.teamcode.testes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TeleOP", group="BahTech")

public class TeleOP extends LinearOpMode {

    // Definição das variáveis
    private final double kp = 0.001;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor EX;
    private DcMotor PC;
    private DcMotor BC;
    private Servo SE;
    private Servo SL;
    private Servo SR;
    ColorSensor RS1;
    ColorSensor RS2;
    boolean triangle = true;
    boolean square = true;
    boolean circle = true;
    boolean cross = true;
    boolean dpad_up = true;
    boolean dpad_down = true;
    boolean dpad_left = true;
    boolean dpad_right = true;
    boolean right_bumper = true;
    boolean left_bumper = true;
    boolean amostra = false;

    // Definição de classes dos motores
    @Override
    public void runOpMode() {
        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        EX = hardwareMap.get(DcMotor.class, "EX");
        PC = hardwareMap.get(DcMotor.class, "PC");
        BC = hardwareMap.get(DcMotor.class, "BC");
        SE = hardwareMap.get(Servo.class, "SE");
        SL = hardwareMap.get(Servo.class, "SL");
        SR = hardwareMap.get(Servo.class, "SR");
        RS1 = hardwareMap.get(ColorSensor.class, "RS1");
        RS2 = hardwareMap.get(ColorSensor.class, "RS2");

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        EX.setDirection(DcMotorSimple.Direction.REVERSE);
        //BC.setDirection(DcMotorSimple.Direction.REVERSE);

        EX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        EX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        //Aqui o código está definindo as ações de acordo com o controle de PS4
        while (opModeIsActive()) {

            SE.setPosition(1);

            telemetry.addData("Posição do braço", PC.getCurrentPosition());
            telemetry.addData("Posição da pinça", BC.getCurrentPosition());
            telemetry.addData("amostra", amostra);
            telemetry.update();

            /*if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                EX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                EX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.left_stick_y != 0) {
                PC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                PC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.right_stick_y != 0) {
                BC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else {
                BC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/


            // Sensores dentro da caixa da extensora
            if (((DistanceSensor) RS1).getDistance(DistanceUnit.CM) < 10 || ((DistanceSensor) RS2).getDistance(DistanceUnit.CM) < 10) {
                amostra = true;
            } else{
                amostra = false;
            }

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

            // Movimentação manual do braço
            /*float powerPC = + gamepad2.left_stick_y;
            float powerBC = + gamepad2.right_stick_y;

            // Movimentação manual da extensora
            float powerEX = gamepad2.left_trigger - gamepad2.right_trigger;*/

            // Aqui o código está definindo a velocidade que cada motor utilizara em seus movimentos
            FL.setPower(powerFL*0.4 + powerFL1*0.8 + powerFL2*0.4 );
            BL.setPower(powerBL*0.4 + powerBL1*0.8 + powerBL2*0.4);
            FR.setPower(powerFR*0.45 + powerFR1*0.85 + powerFR2*0.4);
            BR.setPower(powerBR*0.45 + powerBR1*0.85 + powerBR2*0.4);
            //PC.setPower(powerPC*0.8);
            //BC.setPower(powerBC*0.5);
            //EX.setPower(powerEX*0.8);
            if (EX.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                smoother(EX, 0.8);
            }
            if(PC.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                smoother(PC, 1.0);
            }
            if (BC.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                smoother(BC, 0.5);
            }

           /* if (gamepad2.right_stick_y != 0) {
                BC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                float powerBC = +gamepad2.right_stick_y;
                BC.setPower(powerBC * 0.5);
            } else{
                BC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }*/

            // Subir garra extensora
            if (!gamepad2.triangle) {
                triangle = true;
            }
            if (gamepad2.triangle && triangle && amostra) {
                EX.setTargetPosition(4500);
                EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                triangle = false;
            }

            // Subir garra extensora na altura do clip
            if (!gamepad2.circle) {
                circle = true;
            }
            if (gamepad2.circle && circle) {
                EX.setTargetPosition(2250);
                EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                circle = false;
            }

            // Subir o braço e a pinça
            if (!gamepad2.square) {
                square = true;
            }
            if (gamepad2.square && square) {
                BC.setTargetPosition(0);
                BC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                PC.setTargetPosition(0);
                PC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                square = false;
            }

            // Descer a extensora
            if (!gamepad2.cross) {
                cross = true;
            }
            if (gamepad2.cross && cross) {
                EX.setTargetPosition(0);
                EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                cross = false;
            }

            // Sobe a garra de leve
            if (!gamepad2.dpad_up) {
                dpad_up = true;
            }
            if (gamepad2.dpad_up && dpad_up) {
                EX.setTargetPosition(500);
                EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dpad_up = false;
            }

            // Desce a garra de leve
            if (!gamepad2.dpad_down) {
                dpad_down = true;
            }
            if (gamepad2.dpad_down && dpad_down) {
                EX.setTargetPosition(1250);
                EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dpad_down = false;
            }

            //Largar a amostra
            if (!gamepad2.dpad_left) {
                dpad_left = true;
            }
            if(gamepad2.dpad_left && dpad_left)  {
                SE.setPosition(0);
                sleep(2000);
                SE.setPosition(1);
                dpad_left = false;
            }

            // Baixar o braço e a pinça
            if (!gamepad2.dpad_right) {
                dpad_right = true;
            }
            if (gamepad2.dpad_right && dpad_right) {
                PC.setTargetPosition(1100);
                PC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BC.setTargetPosition(80);
                BC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dpad_right = false;
            }

            // Fechar a pinça
            if (!gamepad2.right_bumper) {
                right_bumper = true;
            }
            if (gamepad2.right_bumper && right_bumper) {
                SR.setPosition(0);
                SL.setPosition(1);
                right_bumper = false;
            }

            // Abrir a pinça
            if (!gamepad2.left_bumper) {
                left_bumper = true;
            }
            if (gamepad2.left_bumper && left_bumper) {
                SR.setPosition(1);
                SL.setPosition(0);
                left_bumper = false;
            }
        }
    }
    public void smoother(DcMotor motor, Double targetVelocity) {
        int targetPos = motor.getTargetPosition();
        int currPos = motor.getCurrentPosition();

        double power = Math.abs(targetPos - currPos) * kp;
        if (power <= 0.003)
            motor.setPower(0);
        else if (power <= targetVelocity)
            motor.setPower(power);
        else
            motor.setPower(targetVelocity);

    }

}

// todo: write your code here