export function set_vec3(dst: Float32Array, x: number, y: number, z: number) {
    dst[0] = x;
    dst[1] = y;
    dst[2] = z;
    return dst;
}

export function set_vec3_gen(dst: Float32Array, dst_index: number, x: number, y: number, z: number) {
    dst[dst_index * 3] = x;
    dst[dst_index * 3 + 1] = y;
    dst[dst_index * 3 + 2] = z;
    return dst;
}

export function copy_vec3_gen(dst: Float32Array, dst_index: number, src: Float32Array, src_index: number) {
    dst[dst_index * 3] = src[src_index * 3];
    dst[dst_index * 3 + 1] = src[src_index * 3 + 1];
    dst[dst_index * 3 + 2] = src[src_index * 3 + 2];
    return dst;
}

export function add_vec3(a: Float32Array, b: Float32Array, dst: Float32Array, dst_index: number) {
    dst[dst_index * 3] = a[0] + b[0];
    dst[dst_index * 3 + 1] = a[1] + b[1];
    dst[dst_index * 3 + 2] = a[2] + b[2];
    return dst;
}

export function add_to_vec3_gen(dst: Float32Array, dst_index: number, b: Float32Array, b_index: number) {
    dst[dst_index * 3] += b[b_index * 3];
    dst[dst_index * 3 + 1] += b[b_index * 3 + 1];
    dst[dst_index * 3 + 2] += b[b_index * 3 + 2];
    return dst;
}

export function sub_vec3(a: Float32Array, b: Float32Array, dst: Float32Array, dst_index: number) {
    dst[dst_index * 3] = a[0] - b[0];
    dst[dst_index * 3 + 1] = a[1] - b[1];
    dst[dst_index * 3 + 2] = a[2] - b[2];
    return dst;
}

export function sub_vec3_gen(a: Float32Array, a_index: number, b: Float32Array, b_index: number, dst: Float32Array, dst_index: number) {
    dst[dst_index * 3] = a[a_index * 3] - b[b_index * 3];
    dst[dst_index * 3 + 1] = a[a_index * 3 + 1] - b[b_index * 3 + 1];
    dst[dst_index * 3 + 2] = a[a_index * 3 + 2] - b[b_index * 3 + 2];
    return dst;
}

export function mul_scalar_vec3(vector: Float32Array, scalar: number, dst: Float32Array) {
    dst[0] = vector[0] * scalar;
    dst[1] = vector[1] * scalar;
    dst[2] = vector[2] * scalar;
    return dst;
}

export function mul_scalar_vec3_gen(vectors: Float32Array, index: number, scalar: number, dst: Float32Array, dst_index: number) {
    dst[dst_index * 3] = vectors[index * 3] * scalar;
    dst[dst_index * 3 + 1] = vectors[index * 3 + 1] * scalar;
    dst[dst_index * 3 + 2] = vectors[index * 3 + 2] * scalar;
    return dst;
}

export function mul_scalar_to_vec3(dst: Float32Array, scalar: number) {
    dst[0] *= scalar;
    dst[1] *= scalar;
    dst[2] *= scalar;
    return dst;
}

export function mul_scalar_to_vec3_gen(dst: Float32Array, dst_index: number, scalar: number) {
    dst[dst_index * 3] *= scalar;
    dst[dst_index * 3 + 1] *= scalar;
    dst[dst_index * 3 + 2] *= scalar;
    return dst;
}

export function cross_vec3(a: Float32Array, b: Float32Array, dst: Float32Array, dst_index: number) {
    dst[dst_index * 3] = a[1] * b[2] - a[2] * b[1];
    dst[dst_index * 3 + 1] = a[2] * b[0] - a[0] * b[2];
    dst[dst_index * 3 + 2] = a[0] * b[1] - a[1] * b[0];
    return dst;
}

export function distance_vectors_vec3(vectors: Float32Array, p1: number, p2: number) {
    const x = vectors[p1 * 3] - vectors[p2 * 3];
    const y = vectors[p1 * 3 + 1] - vectors[p2 * 3 + 1];
    const z = vectors[p1 * 3 + 2] - vectors[p2 * 3 + 2];
    return Math.sqrt(x * x + y * y + z * z);
}

export function length_vec3(vector: Float32Array) {
    return Math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

export function length_vec3_gen(vectors: Float32Array, index: number) {
    const x = vectors[index * 3];
    const y = vectors[index * 3 + 1];
    const z = vectors[index * 3 + 2];
    return Math.sqrt(x * x + y * y + z * z);
}

export function normalize_vec3_gen(vectors: Float32Array, index: number) {
    const l = 1 / length_vec3_gen(vectors, index);
    vectors[index * 3] *= l;
    vectors[index * 3 + 1] *= l;
    vectors[index * 3 + 2] *= l;
    return vectors;
}
