const MimeTypes = ['video/webm;codecs=vp9', 'video/mp4;codecs=avc1', 'video/webm;codecs=vp8', 'video/webm'];
const UrlParamName = 'capture';

export default class CanvasRecorder {
    readonly mime_type: string;
    readonly recorder: MediaRecorder;
    readonly record_time: number;

    constructor(canvas: HTMLCanvasElement, record_time: number) {
        if (window.MediaRecorder !== undefined) {
            const support_type = MimeTypes.find(type => MediaRecorder.isTypeSupported(type));
            if (support_type) {
                this.mime_type = support_type;
                this.recorder = new MediaRecorder(canvas.captureStream(60), { mimeType: support_type });
                this.record_time = record_time;

                const ext = support_type.match(/video\/(\w+);?/i)![1];
                this.recorder.ondataavailable = e => download(e.data, `capture.${ext}`);
            }
        }
    }

    static createFromUrlParams(canvas: HTMLCanvasElement, url: string) {
        const params = new URL(url).searchParams;
        const record_time = parseInt(params.get(UrlParamName) || '', 10);
        if (!isNaN(record_time)) {
            return new CanvasRecorder(canvas, record_time * 1000);
        } else {
            return undefined;
        }
    }

    start() {
        if (this.recorder) {
            setTimeout(() => {
                this.recorder.stop();
            }, this.record_time);
            this.recorder.start();
        }
    }
}

function download(blob: Blob, file_name: string) {
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.style.display = 'none';
    a.href = url;
    a.download = file_name;
    a.click();
    URL.revokeObjectURL(url);
}
