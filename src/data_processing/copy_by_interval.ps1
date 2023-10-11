# Specify the source folder path
$sourceFolderPath = Read-Host "Enter the name of the source folder"

if (-not (Test-Path -Path $sourceFolderPath)) {   
    Write-Error $"Source folder '{$sourceFolderPath}' does not exist"
    exit -1
}

# Prompt the user for the target folder name
$targetFolderPath = Read-Host "Enter the name of the target folder"

# Create the target folder if it doesn't exist
if (-not (Test-Path -Path $targetFolderPath)) {
    New-Item -Path $targetFolderPath -ItemType Directory
    Write-Host "Target folder created: $targetFolderPath"
}

$imageInterval = [int] (Read-Host "Enter the interval to skip")
if ($imageInterval -lt 0) {
    Write-Error "Interval must be positive"
    exit -1
}

# Get the list of files in the source folder
$files = Get-ChildItem $sourceFolderPath

# Loop through the files and copy every second file to the target folder
for ($i = 1; $i -lt $files.Count; $i += $imageInterval) {
    $fileToCopy = $files[$i].FullName
    $destinationPath = Join-Path -Path $targetFolderPath -ChildPath $files[$i].Name
    Copy-Item -Path $fileToCopy -Destination $destinationPath
    Write-Host "Copied: $fileToCopy to $destinationPath"
}

Write-Host "Copy complete."
