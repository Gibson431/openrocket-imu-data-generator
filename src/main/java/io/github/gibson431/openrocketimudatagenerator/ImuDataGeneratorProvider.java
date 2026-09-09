package io.github.gibson431.openrocketimudatagenerator;

import info.openrocket.core.plugin.Plugin;
import info.openrocket.core.simulation.extension.AbstractSimulationExtensionProvider;

@Plugin
public class ImuDataGeneratorProvider extends AbstractSimulationExtensionProvider {

	public ImuDataGeneratorProvider() {
		super(ImuDataGenerator.class, "Reports", "IMU Data Generator (CSV)");
	}

}
