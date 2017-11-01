export class SimpleAllocator<T extends object, F extends () => T> {
    readonly object_creator: F;
    private _usage_count: number;
    private _objects: T[];

    constructor(object_creator: F) {
        this.object_creator = object_creator;
        this._usage_count = 0;
        this._objects = [];
    }

    dispose() {
        this._usage_count = 0;
        this._objects.length = 0;
    }

    get() {
        if (this._usage_count < this._objects.length) {
            return this._objects[this._usage_count++];
        } else {
            const obj = this.object_creator();
            this._objects.push(obj);
            ++this._usage_count;
            return obj;
        }
    }

    release_all() {
        this._usage_count = 0;
    }
}
