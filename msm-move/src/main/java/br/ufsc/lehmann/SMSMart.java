package br.ufsc.lehmann;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.google.common.collect.Streams;
import com.google.common.math.BigIntegerMath;

import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.StopSemantic;
import br.ufsc.core.trajectory.semantic.AttributeType;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.core.trajectory.semantic.Stop;
import br.ufsc.ftsm.base.TrajectorySimilarityCalculator;
import br.ufsc.lehmann.SMSMPartial.SMSM_StopInMove;

public class SMSMart extends TrajectorySimilarityCalculator<SemanticTrajectory> {
	
	private SMSM.SMSM_MoveSemanticParameters moveParams;
	private SMSM.SMSM_StopSemanticParameters stopParams;
	private SMSM_StopInMove stopMoveParams;

	public SMSMart(SMSM.SMSM_MoveSemanticParameters moveParams, SMSM.SMSM_StopSemanticParameters stopParams) {
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

					double unmatchedStopsWeight = 0;
					if(moveA instanceof MergedMove) {
						int mergedStopsCount = ((MergedMove) moveA).getInfo().getMergedStopsCount();
						unmatchedStopsWeight += Math.sqrt(BigIntegerMath.factorial(mergedStopsCount + 1).doubleValue()) / 2;
					} else {
						unmatchedStopsWeight += 0.5;
					}
					if(moveB instanceof MergedMove) {
						int mergedStopsCount = ((MergedMove) moveB).getInfo().getMergedStopsCount();
						unmatchedStopsWeight += Math.sqrt(BigIntegerMath.factorial(mergedStopsCount + 1).doubleValue()) / 2;
					} else {
						unmatchedStopsWeight += 0.5;
					}
					score *= unmatchedStopsWeight;
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
				ret.addAll(mergedMoves(s, s).collect(Collectors.toList()));
			}
		}
		return ret;
	}

	private Stream<? extends Move> mergedMoves(Stop anchor, Stop s) {
		if(s.getNextMove() != null) {
			return Streams.concat(Streams.stream(Optional.ofNullable(anchor == s ? null : new MergedMove(anchor.getNextMove(), s.getNextMove()))), mergedMoves(anchor, s.getNextMove().getEnd()));
		}
		return Collections.<Move>emptyList().stream();
	}

	public static class SMSM_Extended_StopSemanticParameters {
		private SMSM_Extended_DimensionParameters[] dimensions;
		private StopSemantic stopSemantic;
		private double weight;
		public SMSM_Extended_StopSemanticParameters(StopSemantic stopSemantic, SMSM_Extended_DimensionParameters[] dimensions) {
			this(stopSemantic, dimensions, 0.5);
		}
		public SMSM_Extended_StopSemanticParameters(StopSemantic stopSemantic, SMSM_Extended_DimensionParameters[] dimensions, double weight) {
			this.stopSemantic = stopSemantic;
			this.dimensions = dimensions;
			this.weight = weight;
		}
		public boolean match(Move moveA, Move moveB) {
			for (int i = 0; i < dimensions.length; i++) {
				if(!dimensions[i].isSpatial) {
					continue;
				}
				boolean matchStart = false;
				Number threshold = dimensions[i].threshold;
				AttributeType attr = dimensions[i].attr;
				Semantic semantic = dimensions[i].semantic;
				if(moveA.getStart() == null && moveB.getStart() == null) {
					matchStart = true;
				} else if(moveA.getStart() != null && moveB.getStart() != null) {
					matchStart = semantic.match(attr.getValue(moveA.getStart()), attr.getValue(moveB.getStart()), threshold);
				}
				if(!matchStart) {
					return false;
				}
				boolean matchEnd = false;
				if(moveA.getEnd() == null && moveB.getEnd() == null) {
					matchEnd = true;
				} else if(moveA.getEnd() != null && moveB.getEnd() != null) {
					matchEnd = semantic.match(attr.getValue(moveA.getEnd()), attr.getValue(moveB.getEnd()), threshold);
				}
				if(!matchEnd) {
					return false;
				}
			}
			return true;
		}
		public double score(Move moveA, Move moveB) {
			double score = 0.0;
			for (int i = 0; i < dimensions.length; i++) {
				AttributeType attr = dimensions[i].attr;
				Semantic semantic = dimensions[i].semantic;
				double weight = dimensions[i].weight;
				Number t = dimensions[i].threshold;
				if(moveA.getStart() == null && moveB.getStart() == null) {
					score += weight;
				} else if(moveA.getStart() != null && moveB.getStart() != null) {
					Object valueA = attr.getValue(moveA.getStart());
					Object valueB = attr.getValue(moveB.getStart());
					score += (semantic.match(valueA, valueB, t) ? 1 : 0) * weight;
				}
				if(moveA.getEnd() == null && moveB.getEnd() == null) {
					score += weight;
				} else if(moveA.getEnd() != null && moveB.getEnd() != null) {
					Object valueA = attr.getValue(moveA.getEnd());
					Object valueB = attr.getValue(moveB.getEnd());
					score += (semantic.match(valueA, valueB, t) ? 1 : 0) * weight;
				}
			}
			return score / 2;
		}
	}

	public static class SMSM_Extended_MoveSemanticParameters {
		private SMSM_Extended_DimensionParameters[] dimensions;
		private double weight;
		public SMSM_Extended_MoveSemanticParameters(MoveSemantic semantic, SMSM_Extended_DimensionParameters[] dimensions) {
			this(semantic, dimensions, 0.5);
		}
		public SMSM_Extended_MoveSemanticParameters(MoveSemantic semantic, SMSM_Extended_DimensionParameters[] dimensions, double weight) {
			this.dimensions = dimensions;
			this.weight = weight;
		}
		public double score(Move moveA, Move moveB) {
			double score = 0.0;
			for (int i = 0; i < dimensions.length; i++) {
				SMSM_Extended_DimensionParameters d = dimensions[i];
				Semantic s = d.semantic;
				Object valueA = d.attr.getValue(moveA);
				Object valueB = d.attr.getValue(moveB);
				score += (s.match(valueA, valueB, d.computeThreshold(moveA, moveB)) ? 1 : 0) * d.weight;
//				score += s.similarity(valueA, valueB) * d.weight;
			}
			return score;
		}
	}

	public static class SMSM_Extended_DimensionParameters<T> {
		private boolean isSpatial;
		private AttributeType attr;
		private Semantic<T, Number> semantic;
		private Number threshold;
		private double weight;
		public SMSM_Extended_DimensionParameters(Semantic<T, Number> semantic, AttributeType attr, Number threshold, double weight) {
			this(semantic, attr, threshold, weight, false);
		}
		public SMSM_Extended_DimensionParameters(Semantic<T, Number> semantic, AttributeType attr, Number threshold, double weight, boolean isSpatial) {
			this.semantic = semantic;
			this.attr = attr;
			this.threshold = threshold;
			this.weight = weight;
			this.isSpatial = isSpatial;
		}
		public AttributeType getAttr() {
			return attr;
		}
		public Semantic<T, Number> getSemantic() {
			return semantic;
		}
		public double getWeight() {
			return weight;
		}
		public Number computeThreshold(Move moveA, Move moveB) {
			if(!(threshold instanceof ComputableDouble)) {
				return threshold;
			}
			return ((ComputableDouble) threshold).compute(moveA, moveB);
		}
		
	}
}
