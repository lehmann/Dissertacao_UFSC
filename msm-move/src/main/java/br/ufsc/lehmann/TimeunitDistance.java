package br.ufsc.lehmann;

import java.time.Instant;
import java.time.temporal.ChronoUnit;

import br.ufsc.core.trajectory.IDistanceFunction;
import br.ufsc.core.trajectory.TemporalDuration;

public class TimeunitDistance implements IDistanceFunction<TemporalDuration> {

	private ChronoUnit timeunit = ChronoUnit.MINUTES;
	
	public TimeunitDistance() {
		this(ChronoUnit.MINUTES);
	}
	
	public TimeunitDistance(ChronoUnit timeunit) {
		this.timeunit = timeunit;
	}

	@Override
	public double distance(TemporalDuration p, TemporalDuration d) {
		return distance(p.getStart(), d.getStart());
	}

	public double distance(Instant d1, Instant d2) {
		double until = d1.until(d2, timeunit);
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
