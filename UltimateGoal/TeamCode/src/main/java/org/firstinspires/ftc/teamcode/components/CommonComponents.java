package org.firstinspires.ftc.teamcode.components;

import java.util.HashMap;
import java.util.Map;

import lombok.Data;

@Data
public class CommonComponents {
    Map<Class<? extends RobotComponent>, RobotComponent> componentMap = new HashMap<>();

    public void init() {
        componentMap.put(Shooter.class, new Shooter());
        componentMap.put(WobbleArm.class, new WobbleArm());
        componentMap.put(Intake.class, new Intake());

        // initialized each registered component in the component map
        componentMap.values().forEach(component -> component.init());
    }

    public <T extends RobotComponent> T get(Class<T> componentClass) {
        return componentClass.cast(componentMap.get(componentClass));
    }
}
