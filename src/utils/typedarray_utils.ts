// depends on typings/TypedArray.d.ts

export function sub_typed_array<T extends TypedArray>(array: T, start: number, count: number) {
    return array.subarray(start, start + count) as T;
}

export function unique_typed_array<T extends TypedArray>(array: T) {
    return new (array.__proto__.constructor)(new Set(array)) as T;
}

export function get_constructor<T extends TypedArray>(array: T) {
    return (<any>array.__proto__.constructor) as TypedArrayConstructor<T>;
}
