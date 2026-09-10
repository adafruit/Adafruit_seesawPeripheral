param(
  [ValidateSet('full', 'gpio', 'four_encoders', 'address_straps', 'address_straps_inverted', 'uart', 'spi', 'spi_queue', 'metro', 'avr')]
  [string]$Target = 'full',
  [Parameter(Mandatory = $true)]
  [string]$OutputRoot,
  [string]$HostLibrary
)
$ErrorActionPreference = 'Stop'
$libraryRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$targetOutput = Join-Path $OutputRoot $Target
$hostProperties = @()
if ($HostLibrary) { $hostProperties = @('--library', $HostLibrary) }
if ($Target -eq 'metro') {
  & arduino-cli compile --fqbn arduino:avr:uno @hostProperties --build-path $targetOutput (Join-Path $PSScriptRoot 'metro_host')
  exit $LASTEXITCODE
}
if ($Target -eq 'avr') {
  $failed = @()
  foreach ($example in Get-ChildItem (Join-Path $libraryRoot 'examples') -Directory) {
    if ($example.Name -eq 'example_stm32c011') { continue }
    $fqbn = 'megaTinyCore:megaavr:atxy7:chip=1617'
    if (Test-Path (Join-Path $example.FullName '.attiny1616.test.only')) {
      $fqbn = 'megaTinyCore:megaavr:atxy6:chip=1616'
    }
    if (Test-Path (Join-Path $example.FullName '.attiny817.test.only')) {
      $fqbn = 'megaTinyCore:megaavr:atxy7:chip=817'
    }
    Write-Output "Building $($example.Name) for $fqbn"
    & arduino-cli compile --fqbn $fqbn --library $libraryRoot --build-path (Join-Path $targetOutput $example.Name) $example.FullName
    if ($LASTEXITCODE -ne 0) { $failed += $example.Name }
  }
  if ($failed.Count) { throw "Failed examples: $($failed -join ', ')" }
  exit 0
}
$fqbn = 'STMicroelectronics:stm32:GenC0:pnum=GENERIC_C011F6UX,xserial=disabled,usb=none,opt=oslto,dbg=none,rtlib=nano,upload_method=OpenOCDSTLink'
$properties = @('--build-property', 'upload.maximum_size=30720')
if ($Target -eq 'gpio') {
  $properties += @('--build-property', 'compiler.cpp.extra_flags=-DHWTEST_UART=0 -DHWTEST_ENCODER=0 -DHWTEST_IRQ=0')
} elseif ($Target -eq 'four_encoders') {
  $properties += @('--build-property', 'compiler.cpp.extra_flags=-DHWTEST_ENCODERS=4')
} elseif ($Target -eq 'uart') {
  $fqbn = $fqbn.Replace('xserial=disabled', 'xserial=generic')
  $properties += @('--build-property', 'compiler.cpp.extra_flags=-DHWTEST_UART=1')
} elseif ($Target -eq 'spi') {
  $properties += @('--build-property', 'compiler.cpp.extra_flags=-DHWTEST_SPI=1')
} elseif ($Target -eq 'spi_queue') {
  $properties += @('--build-property', 'compiler.cpp.extra_flags=-DHWTEST_SPI=1 -DHWTEST_SPI_QUEUE_HOLD=1')
} elseif ($Target -in @('address_straps', 'address_straps_inverted')) {
  $strapFlags = '-DHWTEST_ENCODER=0 -DHWTEST_IRQ=0 -DCONFIG_ADDR_0_PIN=0 -DCONFIG_ADDR_1_PIN=1 -DCONFIG_ADDR_2_PIN=2 -DCONFIG_ADDR_3_PIN=3'
  if ($Target -eq 'address_straps_inverted') { $strapFlags += ' -DCONFIG_ADDR_INVERTED=1' }
  $properties += @('--build-property', "compiler.cpp.extra_flags=$strapFlags")
}
& arduino-cli compile --fqbn $fqbn --library $libraryRoot @properties --build-path $targetOutput (Join-Path $PSScriptRoot 'c011_firmware')
exit $LASTEXITCODE
