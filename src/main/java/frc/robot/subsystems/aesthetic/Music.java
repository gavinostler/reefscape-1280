package frc.robot.subsystems.aesthetic;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

public class Music {
  public final ParentDevice mID1 = new CoreTalonFX(1, "*");
  public final ParentDevice mID2 = new CoreTalonFX(2, "*");
  public final ParentDevice mID3 = new CoreTalonFX(3, "*");
  public final ParentDevice mID4 = new CoreTalonFX(4, "*");
  public final ParentDevice mID5 = new CoreTalonFX(5, "*");
  public final ParentDevice mID6 = new CoreTalonFX(6, "*");
  public final ParentDevice mID7 = new CoreTalonFX(7, "*");
  public final ParentDevice mID8 = new CoreTalonFX(8, "*");
  Orchestra bach = new Orchestra();

  public Music() {
    bach.addInstrument(mID1, 1);
    bach.addInstrument(mID2, 2);
    bach.addInstrument(mID3, 3);
    bach.addInstrument(mID4, 4);
    bach.addInstrument(mID5, 5);
    bach.addInstrument(mID6, 6);
    bach.addInstrument(mID7, 7);
    bach.addInstrument(mID8, 8);
  }

  public void myWay() {
    var exist = bach.loadMusic("myway.chrp");
    if (!exist.isOK()) {
      bach.play();
    }
  }

  public void starSprangledBanner() {
    var exist = bach.loadMusic("us.chrp");
    if (!exist.isOK()) {
      bach.play();
    }
  }
}
