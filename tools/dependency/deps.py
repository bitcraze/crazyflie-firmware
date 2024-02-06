#!/usr/bin/env python

import argparse
import sys
from deplib.graph import DependencyGraph
import deplib.visualize as visualize


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--print', help='Print dependencies for a file.')

    parser.add_argument('-d', '--dependson', help='Select files that the target(s) depends on', action='store_true')
    parser.add_argument('-u', '--usedby', help='Select files that are used by the target(s)', action='store_true')
    parser.add_argument('-l', '--levels', help='The number of levels of dependencies to add. Negative numbers are ' +
                        'interpreted as all levels.', default=-1)
    parser.add_argument('-f', '--from-to', help='Select files with dependencies from target_1 to target_2. ' +
                        'Example usage, to find all dependencies from utils to other directories, use targets: ' +
                        "src/utils/.* '^src/(?!utils/).*$'", action='store_true')

    parser.add_argument('-t', '--total', help='Output the total number of files in the selected set',
                        action='store_true')
    parser.add_argument('-cc', '--cfiles', help='Output the number of c files in the selected set', action='store_true')
    parser.add_argument('-hh', '--hfiles', help='Output the number of h files in the selected set', action='store_true')
    parser.add_argument('-z', '--zero', help='Assert that the total count is zero, fail if not', action='store_true')

    parser.add_argument('-v', '--visualize', help='Visualize the selected set in a web browser', action='store_true')
    parser.add_argument('-i', '--ignored', help='Print files that were ignored', action='store_true')

    parser.add_argument('targets', help='The files to operate on. The full graph is selected by default.',
                        nargs=argparse.REMAINDER)

    args = parser.parse_args()

    graph = DependencyGraph()
    graph.add_and_process_dir('src')

    subset = DependencyGraph()
    if args.from_to:
        if len(args.targets) < 2:
            print("Error: must have 2 or more targets")
            sys.exit(1)

        from_set = graph.find(args.targets[0])
        to_set = DependencyGraph()
        for target in args.targets[1:]:
            to_set.union(graph.find(target))
        subset = from_set.with_dependency_to(to_set)
    else:
        for target in args.targets:
            subset.union(graph.find(target))

    if args.dependson:
        for target in args.targets:
            subset.union(graph.depends_on(target, int(args.levels)))

    if args.usedby:
        for target in args.targets:
            subset.union(graph.used_by(target, int(args.levels)))

    if args.ignored:
        print('Ignored files:')
        print(graph.get_ignored_files())

    if args.total:
        print(subset.get_file_count())

    if args.cfiles:
        print(subset.get_c_file_count())

    if args.hfiles:
        print(subset.get_h_file_count())

    if args.visualize:
        nodes, edges = subset.export()
        visualize.render(nodes, edges)

    if args.zero:
        count = subset.get_file_count()
        if count != 0:
            print(f'Error: file count is not 0 ({count})', file=sys.stderr)
            sys.exit(1)
