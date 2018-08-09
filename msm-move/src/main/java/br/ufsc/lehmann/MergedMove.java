package br.ufsc.lehmann;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import br.ufsc.core.trajectory.TPoint;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.utils.Angle;

public class MergedMove extends Move {
	
	private MovesInfo info;

	public MergedMove(Move first, Move last) {
		this(first, last, new MovesInfo(first, last));
	}

	private MergedMove(Move first, Move last, MovesInfo info) {
		super(-1, 
				first.getStart(), last.getEnd(), 
				first.getStartTime(), last.getEndTime(), 
				first.getBegin(), info.getGPSPoints().length, 
				info.getGPSPoints(), 
				info.getAngle(), info.getTraveledDistance(), info.getStreetsName(), 
				info.getUser(), info.getDimensaoData());
		this.info = info;
	}

	public MovesInfo getInfo() {
		return info;
	}
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((info == null) ? 0 : info.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		MergedMove other = (MergedMove) obj;
		if (info == null) {
			if (other.info != null)
				return false;
		} else if (!info.equals(other.info))
			return false;
		return true;
	}

	static class MovesInfo {
		private Move first;
		private Move last;
		private TPoint[] points;
		private int moveLength;
		private double traveledDistance;
		private List<String> streetsName;
		private double angle;
		
		private int mergedStopsCount = 0;
		
		public MovesInfo(Move first2, Move last2) {
			this.first = first2;
			this.last = last2;
			this.moveLength = (last2.getBegin() - first2.getBegin()) + last2.getLength();
			this.traveledDistance = 0.0;
			this.streetsName = new ArrayList<>();
			List<TPoint> p = new ArrayList<>(Math.max(0, moveLength));
			while(first2 != last2) {
				p.addAll(Arrays.asList(first2.getPoints()));
				p.addAll(first2.getEnd().getPoints());
				this.traveledDistance += first2.getTravelledDistance()/* + first2.getEnd().getTraveledDistance()*/;
				this.streetsName.add(first2.getStreetName());
				first2 = first2.getEnd().getNextMove();
				mergedStopsCount++;
			}
			points = p.toArray(new TPoint[p.size()]);
			if(points.length > 1) {
				this.angle = Angle.getAngle(points[0], points[points.length - 1]);
			}
		}
		
		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + ((first == null) ? 0 : first.hashCode());
			result = prime * result + ((last == null) ? 0 : last.hashCode());
			result = prime * result + moveLength;
			return result;
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			MovesInfo other = (MovesInfo) obj;
			if (first == null) {
				if (other.first != null)
					return false;
			} else if (!first.equals(other.first))
				return false;
			if (last == null) {
				if (other.last != null)
					return false;
			} else if (!last.equals(other.last))
				return false;
			if (moveLength != other.moveLength)
				return false;
			return true;
		}


		public double getAngle() {
			return this.angle;
		}
		public double getTraveledDistance() {
			return traveledDistance;
		}
		public List<String> getStreetsName() {
			return this.streetsName;
		}
		public Integer getUser() {
			return first.getDimensaoData();
		}
		public Integer getDimensaoData() {
			return first.getDimensaoData();
		}
		public TPoint[] getGPSPoints() {
			return points;
		}
		public int getMergedStopsCount() {
			return mergedStopsCount;
		}
	}
}
