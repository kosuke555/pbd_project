export default [
    {
        entry: 'tmp/src/index.js',
        dest: 'build/bundle.js',
        format: 'iife',
        external: ['three', 'stats.js'],
        globals: { three: 'THREE', 'stats.js': 'Stats' },
        sourceMap: true
    },
    {
        entry: 'tmp/src/index-mmd.js',
        dest: 'build/bundle-mmd.js',
        format: 'iife',
        external: ['three', 'stats.js'],
        globals: { three: 'THREE', 'stats.js': 'Stats' },
        sourceMap: true
    }
];
