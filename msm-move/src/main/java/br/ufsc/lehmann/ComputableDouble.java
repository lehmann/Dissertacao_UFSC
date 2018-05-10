package br.ufsc.lehmann;

import org.apache.commons.lang3.mutable.MutableDouble;

public abstract class ComputableDouble<T> extends MutableDouble {

	public abstract Number compute(T a, T b);

}
