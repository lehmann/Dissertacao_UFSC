package br.ufsc.lehmann;

import br.ufsc.core.trajectory.IDistanceFunction;

public class AngleDistance implements IDistanceFunction<Double> {

	@Override
	public double distance(Double p, Double d) {
		if(p == d) {
			return 0;
		}
		if (p == null || d == null) {
			return Double.MAX_VALUE;
		}
		double phi = Math.abs(p - d) % 360;
		double distance = phi > 180 ? 360 - phi : phi;
		return (distance);
	}

	@Override
	public double convert(double units) {
		return units;
	}

}
