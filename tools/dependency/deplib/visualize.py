
import json
import os
import webbrowser


colors = [
    0xaa0000,
    0x00aa00,
    0x0000aa,
    0x00aaaa,
    0xaa00aa,
    0xaaaa00,
    0xaa6600,
    0xaa0066,
    0x00aa66,
]


def get_color(category: int) -> str:
    base = 0x808080
    if category > 0 and category < (len(colors) + 1):
        base = colors[category - 1]

    return f'#{base:06x}'


def render(nodes: dict[str, dict[str, str | int | bool]], edges: list[list[str]]):
    vis_nodes: list[dict[str, str]] = []
    for file_name, data in nodes.items():
        color = get_color(int(data['category']))

        shape = 'dot'
        if data['is-c-file']:
            shape = 'triangle'

        vis_nodes.append({'id': file_name, 'label': str(data['name']), 'title': file_name, 'shape': shape, 'color': color})

    vis_edges = []
    for edge in edges:
        vis_edges.append({"from": edge[0], "to": edge[1], "arrows": "to"})

    with open(os.path.realpath('tools/dependency/web/data.js'), 'w', encoding='utf8') as f:
        f.write('let nodes = new vis.DataSet(' + json.dumps(vis_nodes) + ');\n')
        f.write('let edges = new vis.DataSet(' + json.dumps(vis_edges) + ');\n')

    webbrowser.open('file://' + os.path.realpath('tools/dependency/web/dependencies.html'))
