package br.ufsc.lehmann;

import br.ufsc.core.trajectory.IDistanceFunction;

public class AngleDistance implements IDistanceFunction<Double> {

	@Override
	public double distance(Double p, Double d) {
		if(p == d) {
			return 0;
		}
		if (p == null || d == null) {
			return 1;
		}
		double phi = Math.abs(p - d) % 360;
		double distance = phi > 180 ? 360 - phi : phi;
		return 1 - (distance / 180);
	}

	@Override
	public double convert(double units) {
		return units;
	}
	
	@Override
	public double maxDistance() {
		return 1;
	}

}
