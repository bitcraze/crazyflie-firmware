import argparse
import struct
import sys

try:
    from elftools.elf.elffile import ELFFile
except ImportError:
    print('pytelftools missing, install to run this script', file=sys.stderr)
    print('https://github.com/eliben/pyelftools#installing', file=sys.stderr)
    sys.exit(1)


class Colors:
    RED = '\033[91m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    END = '\033[0m'


CORE = 0x20

param_type_to_str_dict = {
    0x0 | 0x0 << 2 | 0x1 << 3: 'PARAM_UINT8',
    0x0 | 0x0 << 2 | 0x0 << 3: 'PARAM_INT8',
    0x1 | 0x0 << 2 | 0x1 << 3: 'PARAM_UIN16',
    0x1 | 0x0 << 2 | 0x0 << 3: 'PARAM_INT16',
    0x2 | 0x0 << 2 | 0x1 << 3: 'PARAM_UINT32',
    0x2 | 0x0 << 2 | 0x0 << 3: 'PARAM_INT32',
    0x2 | 0x1 << 2 | 0x0 << 3: 'PARAM_FLOAT'
}


def param_type_to_str(t: int) -> str:
    extra = str()

    if t & (1 << 5):  # PARAM_CORE set
        extra = ' | PARAM_CORE'

    if t & (1 << 6):  # PARAM_RONLY set
        extra += ' | PARAM_RONLY'

    int_type = t & ~(1 << 5 | 1 << 6)

    return '{:12}{}'.format(param_type_to_str_dict[int_type], extra)


log_type_to_str_dict = {
    0x1: 'LOG_UINT8',
    0x2: 'LOG_INT8',
    0x3: 'LOG_UIN16',
    0x4: 'LOG_INT16',
    0x5: 'LOG_UINT32',
    0x6: 'LOG_INT32',
    0x7: 'LOG_FLOAT',
    0x8: 'LOG_FP16'
}


def log_type_to_str(t: int) -> str:
    extra = str()

    if t & (1 << 5):  # LOG_CORE set
        extra = ' | LOG_CORE'

    if t & (1 << 6):  # BY_FUNCTION set
        extra += ' | BY_FUNCTION'

    int_type = t & ~(1 << 5 | 1 << 6)

    return '{:12}{}'.format(log_type_to_str_dict[int_type], extra)


def process_file(filename, list_params: bool, list_logs: bool, core: bool):
    with open(filename, 'rb') as f:
        parameters = check_structs(f, 'param', core)
        if list_params:
            for key in sorted(parameters.keys()):
                t = parameters[key]
                print('{:25}\t{}'.format(key, param_type_to_str(t)))

        logs = check_structs(f, 'log', core)
        if list_logs:
            for key in sorted(logs.keys()):
                t = logs[key]
                print('{:25}\t{}'.format(key, log_type_to_str(t)))

        n_logs = Colors.GREEN + str(len(logs.keys())) + Colors.END
        n_params = Colors.BLUE + str(len(parameters.keys())) + Colors.END
        print('{} parameters and {} log vars in elf'.format(n_params, n_logs))


def get_offset_of(elf, addr):
    for seg in elf.iter_segments():
        if seg.header['p_type'] != 'PT_LOAD':
            continue

        # If the symbol is inside the range of a LOADed segment, calculate the
        # file offset by subtracting the virtual start address and adding the
        # file offset of the loaded section(s)
        if addr >= seg['p_vaddr'] and addr < seg['p_vaddr'] + seg['p_filesz']:
            return addr - seg['p_vaddr'] + seg['p_offset']

    return None


def get_offset_of_symbol(elf, name):
    section = elf.get_section_by_name('.symtab')
    sym = section.get_symbol_by_name(name)[0]
    if not sym:
        print('symbol %s not found' % name, file=sys.stderr)
        sys.exit(1)

    return get_offset_of(elf, sym['st_value'])


def check_structs(stream, what: str, core: bool) -> dict:
    elf = ELFFile(stream)
    offset = get_offset_of_symbol(elf, '_{}_start'.format(what))
    stop_offset = get_offset_of_symbol(elf, '_{}_stop'.format(what))

    name_type_dict = {}
    name_maxlen = 25
    group_bit = 0x1 << 7
    start_bit = 0x1
    if what == 'log':
        struct_len = 12
    else:
        struct_len = 20

    while offset < stop_offset:
        elf.stream.seek(offset)
        #
        # Parsing log or param, first unpack the struct:
        # struct [log_s] {
        #   uint8_t type;
        #   char * name;
        #   void * address;
        # };
        #
        # struct [param_s] {
        #   uint8_t type;
        #   uint8_t extended_type;
        #   char * name;
        #   void * address;
        #   void * callback;
        #   void * getter;
        # };
        #
        # We want the type and the name.
        #
        buffer = elf.stream.read(struct_len)
        if what == 'log':
            t, addr = struct.unpack('@Bxxxixxxx', buffer)
        else:
            t, addr = struct.unpack('@Bxxxixxxxxxxxxxxx', buffer)

        #
        # Next, convert address of name to offset in elf
        #
        addr = get_offset_of(elf, addr)
        #
        # And read the name from that offset
        #
        elf.stream.seek(addr)
        name = ''.join(iter(lambda: stream.read(1).decode('ascii'), '\x00'))
        #
        # Check if this is start of a group
        #
        if t & group_bit != 0 and t & start_bit != 0:
            current_group = name
        elif t & group_bit == 0:
            name = '%s.%s' % (current_group, name)
            if name in name_type_dict:
                print('%sDuplicate parameter detected!%s (%s)' %
                      (Colors.RED, Colors.END, name), file=sys.stderr)
                sys.exit(1)
            else:
                #
                # If core only is specified we check if the core flag is set
                #
                if not core or (t & CORE) != 0:
                    name_type_dict[name] = t

            if len(name) > name_maxlen:
                print('%sName too long!%s (%s > %d)' %
                      (Colors.RED, Colors.END, name, name_maxlen),
                      file=sys.stderr)
                sys.exit(1)

            # Parameter and log names must not contain space as they are mapped to topic in ROS that does not support
            # space.
            if ' ' in name:
                print(f'{Colors.RED}Name contains space(s){Colors.END} ("{name}")', file=sys.stderr)
                sys.exit(1)

        offset += struct_len
    return name_type_dict


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--list-params', action='store_true')
    parser.add_argument('--list-logs', action='store_true')
    parser.add_argument('--core', action='store_true')
    parser.add_argument('filename', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    if args.filename:
        process_file(args.filename[0], args.list_params, args.list_logs, args.core)
    else:
        sys.exit(1)
