package frc.robot.generated;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


// This class is similar to TimedCommand but fixes a bug and it allows for an optional key to be passed in for logging purposes.
// If no key is passed in, the command's name is used as the key.

public class TimedCommand2 extends Command {
  private final Command command;
  private final String key;

  public TimedCommand2(Command command, String key) {
    this.command = command;
    this.key = key;

    setName("Timed" + command.getName());

    addRequirements(command.getRequirements());
  }

  public TimedCommand2(Command command) {
    this.command = command;
    this.key = command.getName();;

    setName("Timed" + command.getName());

    addRequirements(command.getRequirements());
  }


  @Override
  public void initialize() {
    DogLog.log("Commands/" + key + "/.Running", true);
    DogLog.time("Commands/" + key + "/.initialize()");
    command.initialize();
    DogLog.timeEnd("Commands/" + key + "/.initialize()");
    DogLog.time("Commands/" + key + "/.totalExecute()");
  }

  @Override
  public void execute() {
    
    DogLog.time("Commands/" + key + "/.execute()");
    command.execute();
    DogLog.timeEnd("Commands/" + key + "/.execute()");
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return command.runsWhenDisabled();
  }

  @Override
  public void end(boolean interrupted) {
    DogLog.timeEnd("Commands/" + key + "/.totalExecute()");
    var logKey = "Commands/" + key + "/.end(" + interrupted + ")";
    DogLog.time(logKey);
    command.end(interrupted);
    DogLog.timeEnd(logKey);
    DogLog.log("Commands/" + key + "/.Running", false);
  }
}
