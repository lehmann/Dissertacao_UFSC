package br.ufsc.lehmann;

import java.time.Instant;
import java.time.temporal.ChronoUnit;

import br.ufsc.core.trajectory.IDistanceFunction;
import br.ufsc.core.trajectory.TemporalDuration;

public class TimestampDistance implements IDistanceFunction<TemporalDuration> {

	@Override
	public double distance(TemporalDuration p, TemporalDuration d) {
		return distance(p.getStart(), d.getStart());
	}

	public double distance(Instant d1, Instant d2) {
		double until = _distance(d1, d2);
		return 1 - (1 / Math.abs(until));
	}

	private double _distance(Instant d1, Instant d2) {
		double until = d1.until(d2, ChronoUnit.MINUTES);
		if(until == 0.0) {
			return 0.0;
		}
		return until;
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
