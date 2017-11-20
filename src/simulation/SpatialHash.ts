interface HashSlot<T> {
    timestamp: number;
    entries: T[];
}

export interface SpatialHashParams {
    table_size: number;
    grid_cell_size: number;
    position_offset: number;
}

const abs = Math.abs;

export class SpatialHash<T> {
    readonly table_size: number;
    readonly grid_cell_size: number;
    readonly position_offset: number;
    
    private _table: HashSlot<T>[];

    constructor(params: SpatialHashParams) {
        this.table_size = params.table_size;
        this.grid_cell_size = params.grid_cell_size;
        this.position_offset = params.position_offset;
        this._table = new Array(this.table_size);
        for (let i = 0, len = this.table_size; i < len; ++i) {
            this._table[i] = { timestamp: 0, entries: [] };
        }
    }

    hash(x: number, y: number, z: number): number {
        const offset = this.position_offset;
        return ((abs(x + offset) * 73856093) ^ (abs(y + offset) * 19349669) ^ (abs(z + offset) * 83492791)) % this.table_size;
    }

    set(x: number, y: number, z: number, timestamp: number, content: T): void {
        const slot = this._table[this.hash(x, y, z)];
        if (slot.timestamp < timestamp) {
            slot.timestamp = timestamp;
            slot.entries.length = 0;
        }
        slot.entries.push(content);
    }

    get(x: number, y: number, z: number, timestamp: number): T[]|undefined {
        const slot = this._table[this.hash(x, y, z)];
        if (slot.timestamp === timestamp) {
            return slot.entries;
        } else {
            return undefined;
        }
    }
}
