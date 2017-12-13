export default [
    {
        entry: 'tmp/src/index-cloth.js',
        dest: 'docs/pbd/bundle-cloth.js',
        format: 'iife',
        external: ['three', 'stats.js'],
        globals: { three: 'THREE', 'stats.js': 'Stats' },
        sourceMap: true
    },
    {
        entry: 'tmp/src/index-mmd.js',
        dest: 'docs/pbd/bundle-mmd.js',
        format: 'iife',
        external: ['three', 'stats.js', 'ammo.js'],
        globals: { three: 'THREE', 'stats.js': 'Stats', 'ammo.js': 'Ammo' },
        sourceMap: true
    }
];
