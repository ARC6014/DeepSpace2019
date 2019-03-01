package frc.robot.commandgroups;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LiftElevator;

public class CargoShipCargo extends CommandGroup {
    private final double cargoShipHeight = 120;

    public CargoShipCargo() {
        addSequential(new LiftElevator(cargoShipHeight));
    }
}
