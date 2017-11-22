type TypedArray = Int8Array|Uint8Array|Uint8ClampedArray|Int16Array|Uint16Array|Int32Array|Uint32Array|Float32Array|Float64Array;

interface TypedArrayConstructor<T extends TypedArray> {
    readonly prototype: T;
    new(length: number): T;
    new(array: ArrayLike<number>): T;
    new(buffer: ArrayBufferLike, byteOffset?: number, length?: number): T;
    new (elements: Iterable<number>): T;

    /**
      * The size in bytes of each element in the array.
      */
    readonly BYTES_PER_ELEMENT: number;

    /**
      * Returns a new array from a set of elements.
      * @param items A set of elements to include in the new array object.
      */
    of(...items: number[]): T;

    /**
      * Creates an array from an array-like or iterable object.
      * @param arrayLike An array-like or iterable object to convert to an array.
      * @param mapfn A mapping function to call on every element of the array.
      * @param thisArg Value of 'this' used to invoke the mapfn.
      */
    from(arrayLike: ArrayLike<number>, mapfn?: (v: number, k: number) => number, thisArg?: any): T;

    /**
     * Creates an array from an array-like or iterable object.
     * @param arrayLike An array-like or iterable object to convert to an array.
     * @param mapfn A mapping function to call on every element of the array.
     * @param thisArg Value of 'this' used to invoke the mapfn.
     */
    from(arrayLike: Iterable<number>, mapfn: (this: void, v: number, k: number) => number): T;
    from(arrayLike: Iterable<number>, mapfn: (this: void, v: number, k: number) => number, thisArg: undefined): T;
    from<Z>(arrayLike: Iterable<number>, mapfn: (this: Z, v: number, k: number) => number, thisArg: Z): T;

    from(arrayLike: Iterable<number>): T;
}

interface Int8Array {
    constructor: Int8ArrayConstructor;
    __proto__: typeof Int8Array.prototype;
}

interface Uint8Array {
    constructor: Uint8ArrayConstructor;
    __proto__: typeof Uint8Array.prototype;
}

interface Uint8ClampedArray {
    constructor: Uint8ClampedArrayConstructor;
    __proto__: typeof Uint8ClampedArray.prototype;
}

interface Int16Array {
    constructor: Int16ArrayConstructor;
    __proto__: typeof Int16Array.prototype;
}

interface Uint16Array {
    constructor: Uint16ArrayConstructor;
    __proto__: typeof Uint16Array.prototype;
}

interface Int32Array {
    constructor: Int32ArrayConstructor;
    __proto__: typeof Int32Array.prototype;
}

interface Uint32Array {
    constructor: Uint32ArrayConstructor;
    __proto__: typeof Uint32Array.prototype;
}

interface Float32Array {
    constructor: Float32ArrayConstructor;
    __proto__: typeof Float32Array.prototype;
}

interface Float64Array {
    constructor: Float64ArrayConstructor;
    __proto__: typeof Float64Array.prototype;
}
