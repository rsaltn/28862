package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ShootManager {

    public enum State { IDLE, SPIN_UP, READY, TRANSFERRING, RECOVER, DONE }

    private final Shooter shooter;
    private final Sorter sorter;
    private final DcMotor feederMotor; // optional (может быть null)
    private final Telemetry telemetry;

    private State state = State.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime overallTimer = new ElapsedTime();

    private Sorter.detectedColor[] order;
    private int currentIndex = 0;
    private int shotsTarget = 0;

    // Настраиваемые параметры (изменяй под робот)
    private double readyRpmRatio = 0.88;    // считать готовым, если measuredRpm >= readyRpmRatio * targetRpm
    private double betweenShotsDelay = 0.25;
    private double maxSeriesTime = 12.0;    // защита на всю серию
    private double perShotTimeout = 3.0;    // максимальное время на один выстрел

    private boolean active = false;

    public ShootManager(Shooter shooter, Sorter sorter, DcMotor feederMotor, Telemetry telemetry) {
        this.shooter = shooter;
        this.sorter = sorter;
        this.feederMotor = feederMotor;
        this.telemetry = telemetry;
    }

    //    / Запускает серию. order — массив цветов (например {PURPLE, PURPLE, GREEN}). */
    public void startSeries(Sorter.detectedColor[] order, double targetRpm) {
        if (order == null || order.length == 0) {
            this.state = State.DONE;
            this.active = false;
            return;
        }
        this.order = order;
        this.shotsTarget = order.length;
        this.currentIndex = 0;
        this.active = true;

        // установим целевой RPM и включим шутер (shootON управляет как power, так и углом)
        shooter.shootON(new double[] { targetRpm, Shooter.basic_angle });

        this.state = State.SPIN_UP;
        this.stateTimer.reset();
        this.overallTimer.reset();
    }

    public boolean isActive() { return active; }
    public boolean isDone() { return !active; }

    /** Должен вызываться часто (loop). Возвращает true когда серия завершена. */
    public boolean update() {
        if (!active) {
            // держим моторы выключенными для безопасности
            shooter.shootON(new double[] { 0.0, Shooter.basic_angle });
            return true;
        }

        // Поддерживаем раскрутку/управление в каждом цикле — shootON обновляет PI-контроллер
        double currentTarget = shooter.getTargetRpm();
        if (currentTarget > 1.0) {
            shooter.shootON(new double[] { currentTarget, Shooter.basic_angle });
        } else {
            // ничего не меняем — стартSeries должен выставить цель
        }

        // Телеметрия (коротко)
        if (telemetry != null) {
            telemetry.addData("SM_state", state);
            telemetry.addData("SM_idx", "%d/%d", currentIndex, shotsTarget);
            telemetry.addData("SM_meas_rpm", "%.0f", shooter.getMeasuredRpm());
            telemetry.addData("SM_target_rpm", "%.0f", shooter.getTargetRpm());
        }

        // Защита по общему времени
        if (overallTimer.seconds() > maxSeriesTime) {
            finishSeries();
            return true;
        }

        switch (state) {
            case SPIN_UP: {
                double measured = shooter.getMeasuredRpm();
                double target = shooter.getTargetRpm();
                if (target <= 1) {
                    // странная ситуация — завершим
                    finishSeries();
                    break;
                }
                if (measured >= readyRpmRatio * target ||  stateTimer.seconds() >perShotTimeout) {
                    state = State.READY;
                    stateTimer.reset();
                }
                break;
            }

            case READY: {
                if (currentIndex >= shotsTarget) {
                    finishSeries();
                    break;
                }
                // стартуем трансфер нужного цвета
                Sorter.detectedColor desired = order[currentIndex];
                sorter.startTransfer(desired);
                state = State.TRANSFERRING;
                stateTimer.reset();
                break;
            }

            case TRANSFERRING: {
                boolean inProgress = sorter.updateTransfer();
                if (feederMotor != null) {
                    feederMotor.setPower(inProgress ? 1.0 : 0.0);
                }
                if (!inProgress) {
                    // один цикл трансфера завершён — считаем что один выстрел произведён
                    currentIndex++;
                    state = State.RECOVER;
                    stateTimer.reset();
                } else {
                    // защитный таймаут на трансфер
                    if (stateTimer.seconds() > perShotTimeout) {
                        currentIndex++;
                        state = State.RECOVER;
                        stateTimer.reset();
                    }
                }
                break;
            }

            case RECOVER: {
                if (stateTimer.seconds() > betweenShotsDelay) {
                    if (currentIndex >= shotsTarget) {
                        finishSeries();
                    } else {
                        // вернуться и дать шутеру догнать цель
                        state = State.SPIN_UP;
                        stateTimer.reset();
                    }
                }
                break;
            }

            case DONE:
            default: {
                finishSeries();
                break;
            }
        }

        return !active;
    }

    private void finishSeries() {
        // выключаем все приводы и помечаем завершение
        if (feederMotor != null) feederMotor.setPower(0.0);
        shooter.shootON(new double[] { 0.0, Shooter.basic_angle });
        active = false;
        state = State.DONE;
    }
}