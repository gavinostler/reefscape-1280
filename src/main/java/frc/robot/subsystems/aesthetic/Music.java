package frc.robot.subsystems.aesthetic;

import frc.robot.Constants.MidiID;
import frc.robot.Constants.MidiID;


class Music {
    MidiID mid = new MidiID(); 
    Orchestra bach = new Orchestra();
    public Music(){
        bach.addInstrument(mid.mID1,1);
        bach.addInstrument(mid.mID2,2);
        bach.addInstrument(mid.mID3,3);
        bach.addInstrument(mid.mID4,4);
        bach.addInstrument(mid.mID5,5);
        bach.addInstrument(mid.mID6,6);
        bach.addInstrument(mid.mID7,7);
        bach.addInstrument(mid.mID8,8);

    }
    public void myWay(){
        var exist = bach.loadMusic("myway.chrp");
        if(!exist.isOK()){
            bach.play();
        }
    }
}
