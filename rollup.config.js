import resolve from 'rollup-plugin-node-resolve';

export default [
    {
        entry: 'tmp/src/index.js',
        dest: 'build/bundle.js',
        format: 'iife',
        plugins: [resolve({
            customResolveOptions: {
                packageFilter: pkg => {
                    if (pkg.name === 'stats.js') {
                        pkg.main = 'src/Stats.js';
                    } else if (pkg.module) {
                        pkg.main = pkg.module;
                    }
                    return pkg;
                }
            }
        })],
        sourceMap: true
    },
    {
        entry: 'tmp/src/index-mmd.js',
        dest: 'build/bundle-mmd.js',
        format: 'iife',
        external: ['THREE'],
        plugins: [resolve({
            customResolveOptions: {
                packageFilter: pkg => {
                    if (pkg.name === 'stats.js') {
                        pkg.main = 'src/Stats.js';
                    } else if (pkg.module) {
                        pkg.main = pkg.module;
                    }
                    return pkg;
                }
            }
        })],
        sourceMap: true
    }
];
