export function to_enumerable<T extends { length: number }, R>(target: T, accessor: (target: T, index: number) => R) {
    return new Proxy<{ [index: number]: R, [Symbol.iterator]: () => IterableIterator<R>, length: number }>(<any>{}, {
        get(obj, key, receiver) {
            if (typeof key === 'string') {
                const index = parseInt(key, 10);
                if (Number.isInteger(index)) {
                    return accessor(target, index);
                }
                if (key === 'length') {
                    return target.length;
                }
            }

            if (key === Symbol.iterator) {
                return function* () {
                    for (let i = 0, len = target.length; i < len; ++i) {
                        yield accessor(target, i);
                    }
                };
            }

            return Reflect.get(obj, key, receiver);
        }
    });
}
