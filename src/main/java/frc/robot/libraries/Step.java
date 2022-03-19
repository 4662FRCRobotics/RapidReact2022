package frc.robot.libraries;

import java.util.function.BooleanSupplier;

public class Step<K> {
    private K m_name;
    private BooleanSupplier m_enabled;

    public Step(K name, BooleanSupplier enabled) {
        m_name = name;
        m_enabled = enabled;
    }

    public Step(K name) {
        this(name, () -> true);
    }

/*public Step(String name, String refName) {
    this(name, refName.getBoolenSupplier);
}*/



    public boolean isTrue() {
        return this.m_enabled.getAsBoolean(); 
    }

    public K getName() {
        return this.m_name;
    }
}
