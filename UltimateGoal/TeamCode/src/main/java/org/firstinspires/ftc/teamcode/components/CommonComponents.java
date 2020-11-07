package org.firstinspires.ftc.teamcode.components;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CommonComponents {
    Map<Class<? extends RobotComponent>, RobotComponent> componentMap = new HashMap<>();

    public void init() {
        componentMap.put(Shooter.class, new Shooter());

        // initialized each registered component in the component map
        componentMap.values().forEach(component -> component.init());
    }

    protected <T extends RobotComponent> T get(Class<T> componentClass) {
        return componentClass.cast(componentMap.get(componentClass));
    }
}
