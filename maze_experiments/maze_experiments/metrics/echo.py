"""Raw messages as text, the way `ros2 topic echo` prints them (ER9).

Long numeric arrays are summarized as in maze_monitor's raw view (count,
minimum, maximum, mean, first values); long arrays of messages show their
first items.
"""

import math

ARRAY_PREVIEW = 8
LONG_ARRAY = 16
LONG_STRING = 400


def fmt_number(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if math.isnan(v):
            return '.nan'
        if math.isinf(v):
            return '.inf' if v > 0 else '-.inf'
        return repr(v)
    return str(v)


def summarize_array(values):
    n = len(values)
    if n == 0:
        return '[]'
    try:
        vals = [float(x) for x in values]
    except (TypeError, ValueError):
        return f'[{n} values]'
    finite = [v for v in vals if math.isfinite(v)]
    stats = (f'min {min(finite):.6g}  max {max(finite):.6g}  mean '
             f'{sum(finite) / len(finite):.6g}') if finite else 'no finite values'
    if len(finite) < n:
        stats += f'  ({n - len(finite)} inf/nan)'
    head = ', '.join(fmt_number(v) for v in list(values)[:ARRAY_PREVIEW])
    return f'[{n} values]  {stats}  |  {head}, …'


def _is_msg(v):
    return hasattr(v, 'get_fields_and_field_types')


def lines(msg, indent=0):
    pad = '  ' * indent
    out = []
    for name in msg.get_fields_and_field_types():
        v = getattr(msg, name)
        if _is_msg(v):
            out.append(f'{pad}{name}:')
            out += lines(v, indent + 1)
        elif isinstance(v, (list, tuple)) or hasattr(v, 'tolist'):
            seq = v.tolist() if hasattr(v, 'tolist') else list(v)
            if seq and _is_msg(seq[0]):
                out.append(f'{pad}{name}:')
                for item in seq[:ARRAY_PREVIEW]:
                    sub = lines(item, indent + 1)
                    if sub:
                        sub[0] = f'{"  " * indent}- {sub[0].lstrip()}'
                    out += sub
                if len(seq) > ARRAY_PREVIEW:
                    out.append(f'{pad}- … {len(seq) - ARRAY_PREVIEW} more items')
                if not seq:
                    out[-1] += ' []'
            elif len(seq) > LONG_ARRAY:
                out.append(f'{pad}{name}: {summarize_array(seq)}')
            else:
                out.append(f'{pad}{name}:' + (' []' if not seq else ''))
                out += [f'{pad}- {fmt_number(x)}' for x in seq]
        elif isinstance(v, (bytes, bytearray)):
            out.append(f'{pad}{name}: [{len(v)} bytes]')
        elif isinstance(v, str):
            s = v if len(v) <= LONG_STRING else v[:LONG_STRING] + f'… ({len(v)} characters)'
            out.append(f"{pad}{name}: {s!r}" if ('\n' in s or s == '' or ':' in s)
                       else f'{pad}{name}: {s}')
        else:
            out.append(f'{pad}{name}: {fmt_number(v)}')
    return out


def to_text(msg):
    return '\n'.join(lines(msg)) + '\n---'
