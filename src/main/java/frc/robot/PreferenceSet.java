package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

public class PreferenceSet {
    public final String prefix;
    public PreferenceSet(String prefix) {
        this.prefix = prefix;
    }

    
    public double get(String key, double fallback) {
        return Preferences.getDouble(prefix + "/" + key, fallback);
    }

    public void set(String key, double newValue) {
        Preferences.setDouble(prefix + "/" + key, newValue);
    }
}
