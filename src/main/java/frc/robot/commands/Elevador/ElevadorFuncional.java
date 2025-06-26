// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.Tracaocmd;
import frc.robot.subsystems.Intake.Tracao;
import frc.robot.subsystems.ElevadorS;
import java.util.Set;

/**
 * Runs a sequence (Tracaocmd, Subir) only if "tem algo?" is true.
 * If "tem algo?" becomes false during execution, the sequence stops,
 * and a new Subir(0.2) command is scheduled immediately.
 */
public class ElevadorFuncional extends Command {

    private final Command sequenceToRun;
    private final double initialNivel;
    private boolean hasStarted = false; // Flag to track if the sequence actually started
    private boolean conditionWasInitiallyTrue = false; // Track initial state for end() logic

    /**
     * Creates a new ElevadorFuncional.
     *
     * @param nivel The level parameter for the initial Subir command.
     */
    public ElevadorFuncional(double nivel, double Intake) {
        this.initialNivel = nivel;
        Tracao tracao = RobotContainer.tracaoA; // Assuming tracaoA is accessible
        // Assuming Elevador subsystem is needed for Subir command
        ElevadorS elevador = RobotContainer.elevadorS; // Assuming elevador is accessible

        // Define the sequence of commands to run if the condition is true
        // Ensure Subir(nivel) requires the Elevador subsystem
        this.sequenceToRun = new SequentialCommandGroup(
            new Tracaocmd(tracao, Intake).withTimeout(0.2), // Assuming requires Tracao
            new Subir( nivel)      // Assuming requires Elevador
        );

        // Declare requirements based on the sequence that might run.
        // Also add requirements for the Subir(0.2) command that might be scheduled.
        Set<Subsystem> reqs = this.sequenceToRun.getRequirements();
        // Manually add Elevador requirement if Subir(0.2) needs it and it wasn't included
        // Although, Subir(nivel) should already add it.
        // We need to ensure this command requires the subsystems needed by BOTH sequences.
        addRequirements(reqs.toArray(new Subsystem[0]));

        // It's generally safer to explicitly require subsystems needed by commands
        // scheduled within this command's end() method, although the scheduler might handle it.
        // If Subir(0.2) requires Elevador, make sure Elevador is in reqs.
        if (!reqs.contains(elevador)) {
             addRequirements(elevador);
        }
    }

    @Override
    public void initialize() {
        this.conditionWasInitiallyTrue = SmartDashboard.getBoolean("tem algo?", false);
        if (this.conditionWasInitiallyTrue) {
            // If condition is true, initialize the sequence
            this.sequenceToRun.initialize();
            this.hasStarted = true;
            System.out.println("ElevadorFuncional: Iniciando sequência (nível " + initialNivel + ") porque 'tem algo?' é true.");
        } else {
            // If condition is false at the start, do nothing and prepare to finish
            this.hasStarted = false;
            System.out.println("ElevadorFuncional: Não iniciando sequência porque 'tem algo?' é false.");
        }
    }

    @Override
    public void execute() {
        // Only execute the sequence if it has started
        if (this.hasStarted) {
            // isFinished() will check the condition and sequence status
            this.sequenceToRun.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (this.hasStarted) {
            boolean conditionIsCurrentlyFalse = !SmartDashboard.getBoolean("tem algo?", false);
            // Determine if the stop was specifically due to the condition failing
            // (and not just the sequence finishing normally while condition is still true)
            boolean stoppedDueToConditionFail = conditionIsCurrentlyFalse && this.conditionWasInitiallyTrue;

            // End the sequence. It's interrupted if the parent command was interrupted OR if the condition failed.
            this.sequenceToRun.end(interrupted || stoppedDueToConditionFail);

            if (stoppedDueToConditionFail && !interrupted) {
                // If the command ended *because* the condition became false (and wasn't interrupted externally)
                System.out.println("ElevadorFuncional: Condição 'tem algo?' tornou-se false. Cancelando sequência e agendando Subir(0.2).");
                // Schedule the new command
                // Ensure we have the Elevador instance
                Command subirCommand = new Subir( 0.2);
                CommandScheduler.getInstance().schedule(subirCommand);
            } else if (interrupted) {
                 System.out.println("ElevadorFuncional: Sequência interrompida externamente.");
            } else if (!conditionIsCurrentlyFalse && this.sequenceToRun.isFinished()) {
                 System.out.println("ElevadorFuncional: Sequência concluída normalmente enquanto 'tem algo?' era true.");
            } else {
                 // Handles cases like condition being false initially or other edge cases
                 System.out.println("ElevadorFuncional: Comando terminado. Interrompido: " + interrupted + ", Condição Falhou: " + stoppedDueToConditionFail);
            }
        }
        this.hasStarted = false; // Reset flag
        this.conditionWasInitiallyTrue = false; // Reset initial condition tracker
    }

    @Override
    public boolean isFinished() {
        // The command finishes if:
        // 1. It never started (condition was false initially).
        // 2. It started, but the condition became false during execution.
        // 3. It started, the condition remained true, and the sequence finished normally.
        if (!this.hasStarted) {
            return true; // Finished immediately if never started
        } else {
            // Check condition AND sequence status
            boolean conditionOk = SmartDashboard.getBoolean("tem algo?", false);
            boolean sequenceDone = this.sequenceToRun.isFinished();
            // Finish if condition is no longer ok OR if the sequence is done
            return !conditionOk || sequenceDone;
        }
    }
}

