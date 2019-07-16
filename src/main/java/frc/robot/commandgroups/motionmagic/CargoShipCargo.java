package frc.robot.commandgroups.motionmagic;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class CargoShipCargo extends CommandGroup {
    private final double cargoShipHeight = 135;

    public CargoShipCargo() {
        addSequential(new SetElevatorHeight(cargoShipHeight));
    }
}
