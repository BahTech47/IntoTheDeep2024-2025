package org.firstinspires.ftc.teamcode.testes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Auto", group="BahTech")
public class Auto extends LinearOpMode {

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
    IMU imu;
    boolean amostra = false;
    public int delay = 20;

    @Override
    public void runOpMode() throws InterruptedException {

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
        RS1 = hardwareMap.get(ColorSensor.class, "RS1");
        RS2 = hardwareMap.get(ColorSensor.class, "RS2");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        EX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            SE.setPosition(1);

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            imu.initialize(new IMU.Parameters(orientationOnRobot));

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            double currentAngle = orientation.getYaw(AngleUnit.DEGREES);

            telemetry.addData("posição", currentAngle);
            telemetry.addData("amostra", amostra);
            telemetry.addData("encoderFR", FR.getCurrentPosition());
            telemetry.addData("encoderFL", FL.getCurrentPosition());
            telemetry.addData("encoderBR", BR.getCurrentPosition());
            telemetry.addData("encoderBL", BL.getCurrentPosition());
            telemetry.addData("encoderEX", EX.getCurrentPosition());
            telemetry.update();

            // Sensores dentro da caixa da extensora
            if (((DistanceSensor) RS1).getDistance(DistanceUnit.CM) < 10 || ((DistanceSensor) RS2).getDistance(DistanceUnit.CM) < 10) {
                amostra = true;
            } else{
                amostra = false;
            }

            if (amostra) {
                baixarPC();
                lateralMove(true, 80);
                turn(true, 0, 0.3);
                resetEncoder();
                move(280);
                resetEncoder();
                turn(false, 15, 0.3);
                resetEncoder();
                subirEX();
                resetEncoder();
                move(150);
                resetEncoder();
                sleep(100);
                largarAmostra();
                move(-170);
                resetEncoder();
                descerEX();
                turn(true, 0, 0.3);
                resetEncoder();
                rotation(true, 300);
                resetEncoder();
                move(-550);
                resetEncoder();
                rotation(true, 280);
                resetEncoder();
                subirEXpEncostar();
                moveLento(250);
                stope();
            } else {
                move(100);
                resetEncoder();
                baixarPC();
                lateralMove(false, 200);
                resetEncoder();
                turn(true, 0, 0.3);
                resetEncoder();
                subirEXclip();
                move(220);
                resetEncoder();
                descerEX();
                move(-100);
                resetEncoder();
                rotation(true, 300);
                resetEncoder();
                move(-500);
                resetEncoder();
                rotation(false, 300);
                resetEncoder();
                move(-150);
                resetEncoder();
                stope();
            }
        }
    }

    public void smoother(DcMotor motor, Double targetVelocity) {
        int targetPos = motor.getTargetPosition();
        int currPos = motor.getCurrentPosition();
        double power = Math.abs(targetPos - currPos) * kp;
        if (power <= 0.003)
            motor.setPower(0);
        else if (power < targetVelocity) {
            motor.setPower(power);
        }
        else {
            motor.setPower(targetVelocity);
        }
    }

    // Movimentação para frente
    public void move(int position) {
        FL.setTargetPosition(-position);
        FR.setTargetPosition(-position);
        BL.setTargetPosition(position);
        BR.setTargetPosition(position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(FL.getTargetPosition() - FL.getCurrentPosition()) > 10) {
            FL.setPower(0.15);
            FR.setPower(0.15);
            BL.setPower(0.15);
            BR.setPower(0.15);
        }
        sleep(delay);
    }

    public void moveLento(int position) {
        FL.setTargetPosition(-position);
        FR.setTargetPosition(-position);
        BL.setTargetPosition(position);
        BR.setTargetPosition(position);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(FL.getTargetPosition() - FL.getCurrentPosition()) > 10) {
            FL.setPower(0.1);
            FR.setPower(0.1);
            BL.setPower(0.1);
            BR.setPower(0.1);
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

    // Curva em 90°
    public void curve(boolean direction) {
        if (direction) {
            FL.setTargetPosition(-240);
            FR.setTargetPosition(240);
            BL.setTargetPosition(-240);
            BR.setTargetPosition(240);
        } else {
            FL.setTargetPosition(240);
            FR.setTargetPosition(-240);
            BL.setTargetPosition(240);
            BR.setTargetPosition(-240);
        }
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(FL.getTargetPosition() - FL.getCurrentPosition()) > 10) {
            smoother(FL, 0.15);
            smoother(FR, 0.15);
            smoother(BL, 0.15);
            smoother(BR, 0.15);
        }
        telemetry.update();
        sleep(delay);
    }

    // Curva em qualquer angulação
    public void rotation(boolean direction, int position) {
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
            smoother(FL, 0.3);
            smoother(FR, 0.3);
            smoother(BL, 0.3);
            smoother(BR, 0.3);
        }
        telemetry.update();
        sleep(delay);
    }

    // Movimentação lateral
    public void lateralMove(boolean direction, int position) {
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
            smoother(FL, 0.85);
            smoother(FR, 0.8);
            smoother(BL, 0.8);
            smoother(BR, 0.8);
        }
        telemetry.update();
        sleep(delay);
    }

    // Subir a extensora na altura da cesta
    public void subirEX() {
        EX.setTargetPosition(-4500);
        EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(EX.getTargetPosition() - EX.getCurrentPosition()) > 50) {
            smoother(EX, 0.8);
            telemetry.addData("encoderEX", EX.getCurrentPosition());
            telemetry.update();
        }
        sleep(delay);
    }

    // Subir a extensora na altura do clip
    public void subirEXpEncostar() {
        EX.setTargetPosition(-1300);
        EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(EX.getTargetPosition() - EX.getCurrentPosition()) > 10) {
            smoother(EX, 0.8);
        }
        telemetry.update();
        sleep(delay);
    }

    public void subirEXclip() {
        EX.setTargetPosition(-2250);
        EX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(EX.getTargetPosition() - EX.getCurrentPosition()) > 10) {
            smoother(EX, 0.8);
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

    // Largar amostra na cesta
    public  void largarAmostra () {
        SE.setPosition(0);
        sleep(2000);
        SE.setPosition(1);
        sleep(delay);
    }

    // Rotação por IMU
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
        PC.setTargetPosition(1000);
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
        while (Math.abs(PC.getTargetPosition() - PC.getCurrentPosition()) > 10) {
            smoother(PC, 0.8);
        }
        telemetry.update();
        sleep(delay);
    }

    public void stope() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        sleep(999999);
    }
}