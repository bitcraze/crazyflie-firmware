# Rakefile used for running unit tests

HERE = File.expand_path(File.dirname(__FILE__)) + '/'

require 'rake'
require 'rake/clean'
require 'rake/testtask'
require './tools/test/rakefile_helper'

include RakefileHelpers

# Load default configuration, for now
DEFAULT_CONFIG_FILE = './tools/test/gcc.yml'
configure_toolchain(DEFAULT_CONFIG_FILE)

task :unit do
  # This prevents all argumets after 'unit' to be interpreted as targets by rake
  ARGV.each { |a| task a.to_sym do ; end }

  if ARGV.length == 0
    parse_and_run_tests([])
  else
    parse_and_run_tests(ARGV[1..-1])
  end
end

desc "Generate test summary"
task :summary do
  report_summary
end

desc "Build and test Unity"
task :all => [:clean, :unit, :summary]
task :default => [:clobber, :all]
task :ci => [:default]
task :cruise => [:default]

desc "Load configuration"
task :config, :config_file do |t, args|
  configure_toolchain(args[:config_file])
end
