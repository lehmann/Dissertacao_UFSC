package br.ufsc.lehmann;

import br.ufsc.core.trajectory.IDistanceFunction;

public class NumberDistance implements IDistanceFunction<Number> {

	@Override
	public double distance(Number p, Number d) {
		if(p == d) {
			return 0;
		}
		if (p == null || d == null) {
			return Double.MAX_VALUE;
		}
		return Math.abs(p.doubleValue() - d.doubleValue());
	}

	@Override
	public double convert(double units) {
		return units;
	}
	
	@Override
	public double maxDistance() {
		return Double.MAX_VALUE;
	}

}
