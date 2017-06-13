interface CanvasCaptureMediaStreamTrack extends MediaStreamTrack {
    readonly canvas: HTMLCanvasElement;
    requestFrame(): void;
}

interface CanvasCaptureMediaStream extends MediaStream {
    getAudioTracks(): CanvasCaptureMediaStreamTrack[];
    getVideoTracks(): CanvasCaptureMediaStreamTrack[];
    getTracks(): CanvasCaptureMediaStreamTrack[];

    getTrackById(trackId: string): CanvasCaptureMediaStreamTrack;
}

interface HTMLCanvasElement {
    captureStream(frameRate?: number): CanvasCaptureMediaStream;
}

interface BlobEvent extends Event {
    readonly data: Blob;
    readonly timecode: number;
}

interface MediaRecorderErrorEvent extends Event {
    error: DOMException;
    message: string;
}

interface MediaRecorder extends EventTarget {
    readonly mimeType: string;
    readonly state: 'inactive'|'recording'|'paused';
    readonly stream: MediaStream;
    readonly videoBitsPerSecond: number;
    readonly audioBitsPerSecond: number;
    onstart: (ev: Event) => any;
    onstop: (ev: Event) => any;
    ondataavailable: (ev: BlobEvent) => any;
    onpause: (ev: Event) => any;
    onresume: (ev: Event) => any;
    onerror: (ev: MediaRecorderErrorEvent) => any;
    start(timeslice?: number): void;
    stop(): void;
    pause(): void;
    resume(): void;
    requestData(): void;
}

interface MediaRecorderOptions {
    mimeType?: string;
    audioBitsPerSecond?: number;
    videoBitsPerSecond?: number;
    bitsPerSecond?: number;
}

interface MediaRecorderConstructor {
    readonly prototype: MediaRecorder;
    new (stream: MediaStream, options?: MediaRecorderOptions): MediaRecorder;
    isTypeSupported(mimeType: string): boolean;
}

declare const MediaRecorder: MediaRecorderConstructor;

interface Window {
    MediaRecorder: typeof MediaRecorder;
}
