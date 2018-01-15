package br.ufsc.lehmann;

import br.ufsc.core.trajectory.IDistanceFunction;

public class ProportionalDistance implements IDistanceFunction<Double> {
	
	private double baseline;

	public ProportionalDistance(Number baseline) {
		this.baseline = Math.abs(baseline.doubleValue());
	}

	@Override
	public double distance(Double p, Double d) {
		if(p == d) {
			return 0;
		}
		if (p == null || d == null) {
			return Double.MAX_VALUE;
		}
		double abs = Math.abs(p - d);
		return Math.min(1, abs / baseline);
	}

	@Override
	public double convert(double units) {
		return Math.min(1, units / baseline);
	}
	
	@Override
	public double maxDistance() {
		return Double.MAX_VALUE;
	}

}
