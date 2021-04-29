import argparse
import struct
import sys

from elftools.elf.elffile import ELFFile


PARAM_NAME_MAXLEN = 25
PARAM_SIZE = 12
PARAM_GROUP = 0x1 << 7
PARAM_START = 0x1

parameters = {}


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
    read_only = str()
    if t & (1 << 6):  # PARAM_RONLY set
        read_only = ' | PARAM_RONLY'

    return '{:12}{}'.format(param_type_to_str_dict[t & ~(1 << 6)], read_only)


def process_file(filename):
    with open(filename, 'rb') as f:
        check_params(f)


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


def check_params(stream):
    elf = ELFFile(stream)
    offset = get_offset_of_symbol(elf, '_param_start')
    stop_offset = get_offset_of_symbol(elf, '_param_stop')

    while offset < stop_offset:
        elf.stream.seek(offset)
        #
        # Parsing a parameter, first unpack the param_s struct:
        # struct param_s {
        #   uint8_t type;
        #   char * name;
        #   void * address;
        # };
        #
        # We want the type and the name.
        #
        buffer = elf.stream.read(PARAM_SIZE)
        t, addr = struct.unpack('@Bxxxixxxx', buffer)
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
        if t & PARAM_GROUP != 0 and t & PARAM_START != 0:
            current_group = name
        elif t & PARAM_GROUP == 0:
            name = '%s.%s' % (current_group, name)
            if name in parameters:
                print('duplicate parameter detected: %s' % name)
                sys.exit(1)
            else:
                parameters[name] = t

            if len(name) > PARAM_NAME_MAXLEN:
                print('name of param to long (%s > %d)' %
                      (name, PARAM_NAME_MAXLEN))
                sys.exit(1)

        offset += PARAM_SIZE


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--list-params', action='store_true')
    parser.add_argument('filename', nargs=argparse.REMAINDER)
    args = parser.parse_args()

    if args.filename:
        process_file(args.filename[0])
        for key in sorted(parameters.keys()):
            print('{:25}\t{}'.format(key, param_type_to_str(parameters[key])))
    else:
        sys.exit(1)
