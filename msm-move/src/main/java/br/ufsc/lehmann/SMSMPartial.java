package br.ufsc.lehmann;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.geom.PrecisionModel;
import org.locationtech.jts.geom.impl.CoordinateArraySequence;

import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.TPoint;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.core.trajectory.semantic.Stop;
import br.ufsc.ftsm.base.TrajectorySimilarityCalculator;
import br.ufsc.lehmann.SMSM.SMSM_DimensionParameters;
import br.ufsc.lehmann.SMSM.SMSM_MoveSemanticParameters;
import br.ufsc.lehmann.SMSM.SMSM_StopSemanticParameters;

public class SMSMPartial extends TrajectorySimilarityCalculator<SemanticTrajectory> {
	
	private SMSM_MoveSemanticParameters moveParams;
	private SMSM_StopSemanticParameters stopParams;
	private SMSM_StopInMove stopMoveParams;

	public SMSMPartial(SMSM_MoveSemanticParameters moveParams, SMSM_StopSemanticParameters stopParams) {
		if(moveParams == null) {
			throw new IllegalArgumentException("Params can not be null");
		}
		this.moveParams = moveParams;
		this.stopParams = stopParams;
		this.stopMoveParams = new SMSM_StopInMove(//
				Arrays.stream(stopParams.getDimensions()).filter(p -> p.isSpatial()).findFirst().orElse(null),// 
				Arrays.stream(moveParams.getDimensions()).filter(p -> p.isSpatial()).findFirst().orElse(null));
	}

	@Override
	public double getSimilarity(SemanticTrajectory t1, SemanticTrajectory t2) {
		return moveSimilarity(t1, t2);
	}

	public double moveSimilarity(SemanticTrajectory A, SemanticTrajectory B) {
		List<Move> tuplesA = tuples(A);
		List<Move> tuplesB = tuples(B);
		int n = tuplesA.size();
		int m = tuplesB.size();
		if(n == 0 || m == 0) {
			return 0.0;
		}
		double bScore[] = new double[m];
		
		double parityAB = 0.0;
		
		for (int i = 0; i < n; i++) {
			double maxScore = 0.0;
			Move moveA = tuplesA.get(i);
		
			for (int j = 0; j < m; j++) {
				Move moveB = tuplesB.get(j);
				boolean stopMatch = stopParams.match(moveA, moveB);
				double score = stopParams.score(moveA, moveB) * stopParams.getWeight();
				if(stopMatch) {
					score += (moveParams.score(moveA, moveB)) * moveParams.getWeight();
				} else {
					boolean stopInMove = stopMoveParams.match(moveA, moveB);
					if(stopInMove) {
						score += (moveParams.score(moveA, moveB)) * (moveParams.getWeight());
					}
				}
				
				if (score >= maxScore) {
					maxScore = score;
					bScore[j] = maxScore > bScore[j] ? maxScore : bScore[j];
				}
			}
			parityAB += maxScore;
		}
		
		double parityBA = 0;
		for (int j = 0; j < m; j++) {
			parityBA += bScore[j];
		}
		
		double similarity = (parityAB + parityBA) / (n + m);

		return similarity;
	}
	
	private List<Move> tuples(SemanticTrajectory a) {
		List<Move> ret = new ArrayList<Move>();
		for (int i = 0; i < a.length(); i++) {
			Stop s = stopParams.getStopSemantic().getData(a, i);
			if(s != null) {
				if(s.getPreviousMove() != null && !ret.contains(s.getPreviousMove())) {
					ret.add(s.getPreviousMove());
				}
				if(s.getNextMove() != null && !ret.contains(s.getNextMove())) {
					ret.add(s.getNextMove());
				}
			}
		}
		return ret;
	}

	public static class SMSM_StopInMove {
		private SMSM_DimensionParameters<Stop> stop;
		private SMSM_DimensionParameters<Move> move;
		public SMSM_StopInMove(SMSM_DimensionParameters<Stop> stop, SMSM_DimensionParameters<Move> move) {
			this.stop = stop;
			this.move = move;
		}
		public boolean match(Move moveA, Move moveB) {
			boolean crosses = crosses(moveA, moveB.getEnd());
			if(crosses) {
				return true;
			}
			return crosses(moveB, moveA.getEnd());
		}
		private boolean crosses(Move move, Stop stop) {
			TPoint[] pointsMoveA = move.getPoints();
			List<TPoint> pointsEndStopB = new ArrayList<>(stop.getPoints());
			pointsEndStopB.add(stop.getStartPoint());
			GeometryFactory factory = new GeometryFactory(new PrecisionModel(PrecisionModel.FLOATING));
			List<Coordinate> stopCoordinates = pointsEndStopB.stream().map(p -> new Coordinate(p.getX(), p.getY())).collect(Collectors.toList());
			List<Coordinate> moveCoordinates = Arrays.stream(pointsMoveA).map(p -> new Coordinate(p.getX(), p.getY())).collect(Collectors.toList());
			Polygon stopPoly = factory.createPolygon(new CoordinateArraySequence(stopCoordinates.toArray(new Coordinate[stopCoordinates.size()])));
			LineString movePoly = factory.createLineString(new CoordinateArraySequence(moveCoordinates.toArray(new Coordinate[moveCoordinates.size()])));
			boolean crosses = movePoly.crosses(stopPoly);
			if(crosses) {
				return true;
			}
			return false;
		}
		
	}
}
