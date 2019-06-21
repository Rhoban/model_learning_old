library(ggplot2)
library(optparse)

readAndSplitData <- function(dataPath) {
    tagsPerSheet <- 6
    data <- read.csv(dataPath)
    data <- data[order(data$tagId),]
    tagsId <- unique(data$tagId)
    nbElements <- ceiling(max(tagsId)/tagsPerSheet)
    result <- list()
    idx <- 1
    for (sheet in seq(1,nbElements)) {
        offset <- (sheet-1) * tagsPerSheet
        startIdx <- offset
        endIdx <- tagsPerSheet+offset-1
        firstRow <- min(which(data$tagId >= startIdx))
        lastRow <- max(which(data$tagId <= endIdx))
        # Add non empty data sets to results
        if (firstRow <= lastRow) {
            print(paste(firstRow,lastRow))
            result[[idx]] <- data[seq(firstRow,lastRow),]
            idx <- idx + 1
        }
    }
    result
}

plotTagsErrors <- function(dataPath, prefix) {
    splittedData <- readAndSplitData(dataPath)
    for (data in splittedData) {
        g <- ggplot(data, aes(x=errX, y=errY, group=tagId, color=model));
        xAbsMax <- max(-min(data$errX),max(data$errX))
        yAbsMax <- max(-min(data$errY),max(data$errY))
        maxVal <- max(xAbsMax,yAbsMax)
        g <- g + geom_point(size=0.5)
        g <- g + facet_wrap(~tagId,ncol=2)
        g <- g + coord_cartesian(xlim=c(-xAbsMax,xAbsMax), ylim=c(-yAbsMax,yAbsMax))
        g <- g + theme_bw()
        outputFile = paste0(prefix,"tags_errors_",min(data$tagId),"-",max(data$tagId),".png")
        print(outputFile)
        ggsave(outputFile,width=10,height=10)
    }
}

plotVectorsErrors <- function(dataPath, prefix) {
    splittedData <- readAndSplitData(dataPath)
    for (data in splittedData) {
        minTag <- min(data$tagId)
        maxTag <- max(data$tagId)
        data$tagId <- as.factor(data$tagId)# Setting as factor AFTER computing min and max
        g <- ggplot(data, aes(x=pred_x, y=pred_y, xend=obs_x, yend=obs_y, group="aa", color=marker_id));
        xMin <- 0#min(data$obsX,data$predX)
        xMax <- 644#max(data$obsX,data$predX)
        yMin <- 0#min(data$obsY,data$predY)
        yMax <- 482#max(data$obsY,data$predY)
        # Add label: source: prediction: end: observation
        g <- g + geom_segment(arrow=arrow(length=unit(0.1,"cm")))
        g <- g + coord_cartesian(xlim=c(xMin,xMax), ylim=c(yMin,yMax))
        g <- g + scale_y_reverse()
        g <- g + theme_bw()
        outputFile = paste0(prefix,"vectors_",minTag,"-",maxTag,".png")
        print(outputFile)
        ggsave(outputFile, width=10, height=10)
    }
}


# Script OPTIONS
option_list <- list(
    make_option(c("-p","--prefix"), type="character", default="tmp_",
                help="Output image prefix"),
    make_option(c("-t","--type"), type="character", default="errors",
                help="Type of plot [errors,vectors]")
)
parser <- OptionParser(usage="%prog [options] <logFile>",
                       option_list = option_list)

args <- commandArgs(TRUE)

# Read Options
cmd <- parse_args(parser, args, positional_arguments=1)

input_file <- cmd$args[1]
type <- cmd$option$type
prefix <- cmd$option$prefix

if (type == "errors") {
    plotTagsErrors(input_file, prefix)
} else if (type == "vectors") {
    plotVectorsErrors(input_file, prefix)
} else {
    print(paste0("Invalid option for type: '",type,"'"))
}
