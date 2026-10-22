import argparse
import struct
import sys


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

    if t & (1 << 8):  # PARAM_PERSISTENT set (in extended_type)
        extra += ' | PARAM_PERSISTENT'

    int_type = t & 0x0F  # PARAM_TYPE_MASK

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


class ElfError(Exception):
    pass


class ElfFile:
    """
    Minimal reader for 32-bit little-endian ELF files, using only the standard
    library. It reads the program headers and the symbol table, which is all
    that is needed to find the log and param tables in the firmware.
    """

    PT_LOAD = 1

    def __init__(self, stream):
        self.stream = stream
        self._symbols = None

        self.stream.seek(0)
        if self.stream.read(4) != b'\x7fELF':
            raise ElfError('not an ELF file')
        ident = self._read_at(0, 16)
        if ident[4] != 1 or ident[5] != 1:
            raise ElfError('not a 32-bit little-endian ELF file')

        (_, _, _, _, e_phoff, e_shoff, _, _, e_phentsize, e_phnum,
         e_shentsize, e_shnum, e_shstrndx) = struct.unpack('<HHIIIIIHHHHHH', self._read_at(16, 36))

        self.segments = []
        for i in range(e_phnum):
            p_type, p_offset, p_vaddr, _, p_filesz = struct.unpack(
                '<IIIII', self._read_at(e_phoff + i * e_phentsize, 20))
            if p_type == self.PT_LOAD:
                self.segments.append((p_vaddr, p_filesz, p_offset))

        self.sections = []
        for i in range(e_shnum):
            sh_name, _, _, _, sh_offset, sh_size, sh_link, _, _, sh_entsize = struct.unpack(
                '<IIIIIIIIII', self._read_at(e_shoff + i * e_shentsize, 40))
            self.sections.append({'name_offset': sh_name, 'offset': sh_offset, 'size': sh_size,
                                  'link': sh_link, 'entsize': sh_entsize})

        if e_shstrndx >= len(self.sections):
            raise ElfError('section name string table not found')
        shstrtab = self.sections[e_shstrndx]
        for section in self.sections:
            section['name'] = self.read_string(shstrtab['offset'] + section['name_offset'])

    def _read_at(self, offset, size):
        self.stream.seek(offset)
        data = self.stream.read(size)
        if len(data) != size:
            raise ElfError('unexpected end of file at offset 0x{:x}'.format(offset))
        return data

    def read_string(self, offset):
        """Read a null-terminated ASCII string at a file offset"""
        self.stream.seek(offset)
        chars = []
        while True:
            c = self.stream.read(1)
            if not c:
                raise ElfError('unexpected end of file in string at offset 0x{:x}'.format(offset))
            if c == b'\x00':
                try:
                    return b''.join(chars).decode('ascii')
                except UnicodeDecodeError:
                    raise ElfError('string at offset 0x{:x} is not ASCII'.format(offset))
            chars.append(c)

    def get_section_by_name(self, name):
        for section in self.sections:
            if section['name'] == name:
                return section
        return None

    def get_symbol_value(self, name):
        if self._symbols is None:
            self._symbols = self._read_symbols()
        if name not in self._symbols:
            raise ElfError('symbol {} not found'.format(name))
        return self._symbols[name]

    def _read_symbols(self):
        symtab = self.get_section_by_name('.symtab')
        if symtab is None:
            raise ElfError('no .symtab section (is the file stripped?)')
        if symtab['link'] >= len(self.sections):
            raise ElfError('string table of .symtab not found')
        strtab = self.sections[symtab['link']]
        strings = self._read_at(strtab['offset'], strtab['size'])
        entries = self._read_at(symtab['offset'], symtab['size'])

        symbols = {}
        entsize = symtab['entsize'] or 16
        for i in range(len(entries) // entsize):
            st_name, st_value = struct.unpack_from('<II', entries, i * entsize)
            end = strings.find(b'\x00', st_name)
            if st_name != 0 and end >= 0:
                symbols.setdefault(strings[st_name:end].decode('ascii', 'replace'), st_value)
        return symbols


def process_file(filename, list_params: bool, list_logs: bool, core: bool):
    with open(filename, 'rb') as f:
        elf = ElfFile(f)
        parameters = check_structs(elf, 'param', core)
        if list_params:
            for key in sorted(parameters.keys()):
                t = parameters[key]
                print('{:25}\t{}'.format(key, param_type_to_str(t)))

        logs = check_structs(elf, 'log', core)
        if list_logs:
            for key in sorted(logs.keys()):
                t = logs[key]
                print('{:25}\t{}'.format(key, log_type_to_str(t)))

        n_logs = Colors.GREEN + str(len(logs.keys())) + Colors.END
        n_params = Colors.BLUE + str(len(parameters.keys())) + Colors.END
        print('{} parameters and {} log vars in elf'.format(n_params, n_logs))


def get_offset_of(elf, addr):
    for p_vaddr, p_filesz, p_offset in elf.segments:
        # If the symbol is inside the range of a LOADed segment, calculate the
        # file offset by subtracting the virtual start address and adding the
        # file offset of the loaded section(s)
        if addr >= p_vaddr and addr < p_vaddr + p_filesz:
            return addr - p_vaddr + p_offset

    raise ElfError('address 0x{:08x} is not in a loaded segment'.format(addr))


def get_offset_of_symbol(elf, name):
    return get_offset_of(elf, elf.get_symbol_value(name))


def check_structs(elf, what: str, core: bool) -> dict:
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
        # We want the type and the name. For params, extended_type is added
        # as the upper byte of the type, as in the TYPE of PARAM_ADD_FULL().
        #
        buffer = elf.stream.read(struct_len)
        if what == 'log':
            t, addr = struct.unpack('@Bxxxixxxx', buffer)
        else:
            t, extended_type, addr = struct.unpack('@BBxxixxxxxxxxxxxx', buffer)
            t |= extended_type << 8

        #
        # Next, convert address of name to offset in elf
        #
        addr = get_offset_of(elf, addr)
        #
        # And read the name from that offset
        #
        name = elf.read_string(addr)
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

    if not args.filename:
        parser.error('no ELF file given')

    filename = args.filename[0]
    try:
        process_file(filename, args.list_params, args.list_logs, args.core)
    except (OSError, ElfError) as e:
        print('{}ELF check failed!{} {}: {}'.format(Colors.RED, Colors.END, filename, e), file=sys.stderr)
        sys.exit(1)
