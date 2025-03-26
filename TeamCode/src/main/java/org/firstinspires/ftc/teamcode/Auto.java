package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Auto", group="BahTech")
public class Auto extends LinearOpMode {

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
    private Servo SC;
    ColorSensor RS1;
    ColorSensor RS2;
    IMU imu;
    boolean amostra = false;
    public int delay = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map de todos os componentes
        imu = hardwareMap.get(IMU.class, "imu");
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
        SC = hardwareMap.get(Servo.class, "SC");
        RS1 = hardwareMap.get(ColorSensor.class, "RS1");
        RS2 = hardwareMap.get(ColorSensor.class, "RS2");

        // Reset de todos os encoders
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Código para os motores das garras travarem quando estiverem em potência zero
        EX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // Posição inicial dos servos
            SE.setPosition(1);

            // Definições iniciais do IMU
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

            // Feedbacks por telemetria
            telemetry.addData("posição", currentAngle);
            telemetry.addData("amostra", amostra);
            telemetry.addData("BC", BC.getCurrentPosition());
            telemetry.update();

            // Sensores dentro da caixa da extensora
            amostra = ((DistanceSensor) RS1).getDistance(DistanceUnit.CM) < 10 || ((DistanceSensor) RS2).getDistance(DistanceUnit.CM) < 10;

            // Instruções para as duas opções de estratégias (basquete e clip, respectivamente)
            if (amostra) {
                // Largar primeira amostra na cesta alta:
                baixarPC();
                abrirPinça();
                baixarBC();
                lateralMove(true, 50, 0.8);
                turn(true, 0, 0.3);
                resetEncoder();
                move(250, 0.5);
                resetEncoder();
                subirEX();
                resetEncoder();
                move(120, 0.5);
                resetEncoder();
                sleep(500);
                largarAmostra();
                move(-80, 0.5);
                resetEncoder();
                descerEX();
                // Pegar segunda amostra:
                rotation(true, 305, 0.8);
                resetEncoder();
                move(-185, 0.5);
                resetEncoder();
                baixarPC2();
                sleep(500);
                fecharPinça();
                sleep(500);
                girarPinça();
                levantarPC();
                abrirPinça();
                sleep (500);
                // Largar segunda amostra na cesta alta ou estacionar:
                amostra = ((DistanceSensor) RS1).getDistance(DistanceUnit.CM) < 10 || ((DistanceSensor) RS2).getDistance(DistanceUnit.CM) < 10;
                telemetry.addData("amostra", amostra);
                telemetry.update();
                if(!amostra) {
                    baixarBC();
                    lateralMove(false, 200, 0.8);
                    resetEncoder();
                    move(-450, 0.5);
                    resetEncoder();
                    rotation(true, 305, 0.8);
                    resetEncoder();
                    move(-30, 0.5);
                    resetEncoder();
                    subirEXpEncostar();
                    move(100, 0.5);
                    FL.setPower(0.2);
                    FR.setPower(0.2);
                    BL.setPower(0.2);
                    BR.setPower(0.2);
                    sleep(999999);
                } else {
                    baixarPC();
                    move(100, 0.5);
                    resetEncoder();
                    rotation(false, 220, 0.8);
                    resetEncoder();
                    subirEX();
                    move(110, 0.5);
                    resetEncoder();
                    largarAmostra();
                    move(-100, 0.5);
                    descerEX();
                    stope();
                }
            } else {
                // Largar primeiro clip:
                SubirCesta();
                move(150, 0.5);
                resetEncoder();
                subirEXclip();
                move(210, 0.2);
                resetEncoder();
                descerEX();
                // Pegar segundo clip do jogador humano:
                move(-130, 0.5);
                resetEncoder();
                rotation(true, 320, 0.8);
                resetEncoder();
                move(-450, 0.5);
                resetEncoder();
                rotation(true, 280, 0.3);
                resetEncoder();
                moveRápido(200);
                resetEncoder();
                subirEXclip();
                // Largar segundo clip:
                move(-150, 0.5);
                resetEncoder();
                rotation(true, 285, 0.3);
                resetEncoder();
                move(-500, 0.5);
                resetEncoder();
                rotation(true, 280, 0.3);
                resetEncoder();
                turn(true, 0, 0.5);
                resetEncoder();
                descerEX();
                // Estacionar:
                move(-150, 0.5);
                resetEncoder();
                rotation(true, 250, 0.8);
                resetEncoder();
                moveRápido(-300);
                resetEncoder();
                stope();
            }
        }
    }

    // Função smoother para os movimentos com encoder e IMU
    public void smoother(DcMotor motor, Double targetVelocity) {
        int targetPos = motor.getTargetPosition();
        int currPos = motor.getCurrentPosition();
        double power = Math.abs(targetPos - currPos) * kp;
        if (power <= 0.000)
            motor.setPower(0);
        else if (power < targetVelocity) {
            motor.setPower(power);
        }
        else {
            motor.setPower(targetVelocity);
        }
    }

    // Movimentação para frente e para trás
    public void move(int position, double power) {
        FL.setTargetPosition(-position);
        FR.setTargetPosition(-position);
        BL.setTargetPosition(position);
        BR.setTargetPosition(position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(FL.getTargetPosition() - FL.getCurrentPosition()) > 10) {
            smoother(FL, power);
            smoother(FR, power);
            smoother(BL, power);
            smoother(BR, power);
        }
        sleep(delay);
    }

    // Movimentação p/ frente e trás de forma bruta
    public void moveRápido(int position) {
        FL.setTargetPosition(-position);
        FR.setTargetPosition(-position);
        BL.setTargetPosition(position);
        BR.setTargetPosition(position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(FL.getTargetPosition() - FL.getCurrentPosition()) > 10) {
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);
        }
        sleep(delay);
    }

    // Resetar os encoders
    public void resetEncoder() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();
        sleep(delay);
    }

    // Curva em qualquer angulação (usando encoders)
    public void rotation(boolean direction, int position, double power) {
        if (direction) {
            FL.setTargetPosition(-position);
            FR.setTargetPosition(position);
            BL.setTargetPosition(position);
            BR.setTargetPosition(-position);
        } else {
            FL.setTargetPosition(position);
            FR.setTargetPosition(-position);
            BL.setTargetPosition(-position);
            BR.setTargetPosition(position);
        }
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(FL.getTargetPosition() - FL.getCurrentPosition()) > 10) {
            smoother(FL, power);
            smoother(FR, power);
            smoother(BL, power);
            smoother(BR, power);
        }
        telemetry.update();
        sleep(delay);
    }

    // Movimentação lateral
    public void lateralMove(boolean direction, int position, double power) {
        if (direction) {
            FL.setTargetPosition(-position);
            FR.setTargetPosition(position);
            BL.setTargetPosition(-position);
            BR.setTargetPosition(position);
        } else {
            FL.setTargetPosition(position);
            FR.setTargetPosition(-position);
            BL.setTargetPosition(position);
            BR.setTargetPosition(-position);
        }
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(FL.getTargetPosition() - FL.getCurrentPosition()) > 10) {
            FL.setPower(power);
            FR.setPower(power+0.05);
            BL.setPower(power);
            BR.setPower(power);
        }
        telemetry.update();
        sleep(delay);
    }

    // Subir a extensora na altura da cesta
    public void subirEX() {
        EX.setTargetPosition(-4500);
        EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(EX.getTargetPosition() - EX.getCurrentPosition()) > 50) {
            smoother(EX, 1.0);
        }
        sleep(delay);
    }

    // Subir a extensora na altura para encostar na barra
    public void subirEXpEncostar() {
        EX.setTargetPosition(-1300);
        EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(EX.getTargetPosition() - EX.getCurrentPosition()) > 10) {
            smoother(EX, 1.0);
        }
        telemetry.update();
        sleep(delay);
    }

    // Subir a extensora na altura para largar o clip
    public void subirEXclip() {
        EX.setTargetPosition(-2250);
        EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(EX.getTargetPosition() - EX.getCurrentPosition()) > 10) {
            smoother(EX, 1.0);
        }
        telemetry.update();
        sleep(delay);
    }

    // Descer a extensora
    public void descerEX() {
        EX.setTargetPosition(0);
        EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(EX.getTargetPosition() - EX.getCurrentPosition()) > 10) {
            smoother(EX, 0.8);
        }
        telemetry.update();
        sleep(delay);
    }

    // subir cesta
    public void SubirCesta () {
        SE.setPosition(0.5);
    }

    // Largar amostra na cesta
    public  void largarAmostra () {
        SE.setPosition(0);
        sleep(1000);
        SE.setPosition(1);
        sleep(delay);
    }

    // Rotação em qualquer angulação (por IMU)
    public void turn (boolean direction, double targetAngle, double power) {
        final double limite = 3;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentAngle = orientation.getYaw(AngleUnit.DEGREES);
        double angle;

        if (direction) {
            angle = -targetAngle + currentAngle;
            if (angle < -180) angle += 360;
            if (angle - currentAngle > 180) currentAngle += 360;

            while (Math.abs(angle-currentAngle) > limite) {
                orientation = imu.getRobotYawPitchRollAngles();
                currentAngle = orientation.getYaw(AngleUnit.DEGREES);
                if (angle - currentAngle > 180) currentAngle += 360;
                smoother(FL, -power);
                smoother(FR, power);
                smoother(BL, -power);
                smoother(BR, -power);
                telemetry.addData("posição", currentAngle);
                telemetry.update();
            }
        } else {
            angle = targetAngle + currentAngle;
            if (angle > 180) angle -= 360;
            if (currentAngle - angle > 180) currentAngle -= 360;

            while (Math.abs(angle-currentAngle) > limite) {
                orientation = imu.getRobotYawPitchRollAngles();
                currentAngle = orientation.getYaw(AngleUnit.DEGREES);
                if (currentAngle - angle > 180) currentAngle -= 360;
                smoother(FL, -power);
                smoother(FR, power);
                smoother(BL, -power);
                smoother(BR, -power);
                telemetry.addData("posição", currentAngle);
                telemetry.update();
            }
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        telemetry.update();
        sleep(delay);
    }

    // Baixar braço
    public void baixarPC () {
        PC.setTargetPosition(500);
        PC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(PC.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            smoother(PC, 1.0);
        }
        telemetry.update();
        sleep(delay);
    }

    public void baixarPC2 () {
        PC.setTargetPosition(700);
        PC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(PC.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            smoother(PC, 1.0);
        }
        telemetry.update();
        sleep(delay);
    }

    // Levantar braço
    public void levantarPC () {
        PC.setTargetPosition(0);
        PC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BC.setTargetPosition(0);
        BC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(PC.getTargetPosition() - PC.getCurrentPosition()) > 10) {
            PC.setPower(1.0);
        }
        while (Math.abs(BC.getTargetPosition() - BC.getCurrentPosition()) > 10) {
            smoother(BC, 0.3);
        }
        telemetry.update();
        sleep(delay);
    }

    public void baixarBC () {
        BC.setTargetPosition(170);
        BC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        smoother(BC, 0.3);
        telemetry.update();
        sleep(delay);
    }

    // Fechar pinça
    public void fecharPinça () {
        SR.setPosition(1);
        SL.setPosition(0);
        telemetry.update();
        sleep(delay);
    }

    // Abrir pinça
    public void abrirPinça () {
        SR.setPosition(0);
        SL.setPosition(1);
        telemetry.update();
        sleep(delay);
    }

    // Girar a pinça
    public void girarPinça () {
        SC.setPosition(0);
    }

    // Parar o robô
    public void stope() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        sleep(999999);
    }

    public void stope2() {
        FL.setPower(0.5);
        FR.setPower(0.5);
        BL.setPower(0.4);
        BR.setPower(0.4);
        sleep(9999999);
    }
}