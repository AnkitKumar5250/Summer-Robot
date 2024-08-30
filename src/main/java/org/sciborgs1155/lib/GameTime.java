package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;

/** Records total elapsed time since the start. */
public class GameTime {
    /** Timer class.*/
    private static final Timer timer = new Timer();

    /** Starts the timer.*/
    public void start() {
        timer.start();
    }

    /** Stops the timer.*/
    public void stop() {
        timer.stop();
    }

    /**
     * Gets the current time.
     * 
     * @return Time.
     */
    public Measure<Time> getTime() {
        return Seconds.of(timer.get());
    }
}
