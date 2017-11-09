package br.ufsc.lehmann;

import java.util.ArrayList;
import java.util.List;

import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.StopSemantic;
import br.ufsc.core.trajectory.semantic.AttributeType;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.ftsm.base.TrajectorySimilarityCalculator;

public class H_MSM_StopMove extends TrajectorySimilarityCalculator<SemanticTrajectory> {
	
	private H_MSM_MoveSemanticParameters moveParams;
	private H_MSM_StopSemanticParameters stopParams;

	public H_MSM_StopMove(H_MSM_MoveSemanticParameters moveParams, H_MSM_StopSemanticParameters stopParams) {
		if(moveParams == null) {
			throw new IllegalArgumentException("Params can not be null");
		}
		this.moveParams = moveParams;
		this.stopParams = stopParams;
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
		double bScore[] = new double[m];
		
		double parityAB = 0.0;
		
		for (int i = 0; i < n; i++) {
			double maxScore = 0.0;
			Move moveA = tuplesA.get(i);
		
			for (int j = 0; j < m; j++) {
				Move moveB = tuplesB.get(j);
				boolean stopMatch = stopParams.match(moveA, moveB);
				double score = stopParams.score(moveA, moveB) * stopParams.weight;
				if(stopMatch) {
					score += (moveParams.score(moveA, moveB)) * moveParams.weight;
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
			Move move = moveParams.getMoveSemantic().getData(a, i);
			if(move != null) {
				ret.add(move);
			}
		}
		return ret;
	}

	public static class H_MSM_StopSemanticParameters {
		private H_MSM_DimensionParameters[] dimensions;
		private StopSemantic stopSemantic;
		private double weight;
		public H_MSM_StopSemanticParameters(StopSemantic stopSemantic, H_MSM_DimensionParameters[] dimensions) {
			this(stopSemantic, dimensions, 0.5);
		}
		public H_MSM_StopSemanticParameters(StopSemantic stopSemantic, H_MSM_DimensionParameters[] dimensions, double weight) {
			this.stopSemantic = stopSemantic;
			this.dimensions = dimensions;
			this.weight = weight;
		}
		public boolean match(Move moveA, Move moveB) {
			for (int i = 0; i < dimensions.length; i++) {
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
				if(moveA.getStart() == null && moveB.getStart() == null) {
					score += weight;
				} else if(moveA.getStart() != null && moveB.getStart() != null) {
					score += semantic.similarity(attr.getValue(moveA.getStart()), attr.getValue(moveB.getStart()), dimensions[i].threshold) * weight;
				}
				if(moveA.getEnd() == null && moveB.getEnd() == null) {
					score += weight;
				} else if(moveA.getEnd() != null && moveB.getEnd() != null) {
					score += semantic.similarity(attr.getValue(moveA.getEnd()), attr.getValue(moveB.getEnd()), dimensions[i].threshold) * weight;
				}
			}
			return score / 2;
		}
	}

	public static class H_MSM_MoveSemanticParameters {
		private MoveSemantic semantic;
		private H_MSM_DimensionParameters[] dimensions;
		private double weight;
		public H_MSM_MoveSemanticParameters(MoveSemantic semantic, H_MSM_DimensionParameters[] dimensions) {
			this(semantic, dimensions, 0.5);
		}
		public H_MSM_MoveSemanticParameters(MoveSemantic semantic, H_MSM_DimensionParameters[] dimensions, double weight) {
			this.semantic = semantic;
			this.dimensions = dimensions;
			this.weight = weight;
		}
		public MoveSemantic getMoveSemantic() {
			return semantic;
		}
		public double score(Move moveA, Move moveB) {
			double score = 0.0;
			for (int i = 0; i < dimensions.length; i++) {
				score += dimensions[i].semantic.similarity(dimensions[i].attr.getValue(moveA), dimensions[i].attr.getValue(moveB), dimensions[i].threshold) * dimensions[i].weight;
			}
			return score;
		}
	}

	public static class H_MSM_DimensionParameters<T> {
		private AttributeType attr;
		private Semantic<T, Number> semantic;
		private Number threshold;
		private double weight;
		public H_MSM_DimensionParameters(Semantic<T, Number> semantic, AttributeType attr, Number threshold, double weight) {
			this.semantic = semantic;
			this.attr = attr;
			this.threshold = threshold;
			this.weight = weight;
		}
		public AttributeType getAttr() {
			return attr;
		}
		public Semantic<T, Number> getSemantic() {
			return semantic;
		}
		public Number getThreshold() {
			return threshold;
		}
		public double getWeight() {
			return weight;
		}
		
	}
}
